#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Sensor.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h> // Using Double_V for a 2-vector of servo lengths

#include <gz/math/Vector3.hh>
#include <gz/math/Filter.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class TVCPlugin : public System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
{
public:
    TVCPlugin() :
        thrust_(250.0),
        servo1_current_len_(0.0),
        servo2_current_len_(0.0),
        servo1_target_len_(0.0),
        servo2_target_len_(0.0)
    {
    }

    // ISystemConfigure
    virtual void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override
    {
        this->model_ = Model(_entity);
        if (!this->model_.Valid(_ecm))
        {
            gzerr << "TVCPlugin must be attached to a model entity." << std::endl;
            return;
        }

        // --- Get parameters from SDF ---
        this->thrust_ = _sdf->Get<double>("thrust", this->thrust_).first;
        this->servo_max_speed_ = _sdf->Get<double>("servo_max_speed", 0.05).first; // m/s
        
        // TODO: Get Kinematic parameters from SDF (anchor points, clevis points, etc.)
        // this->servo1_anchor_pos_ = _sdf->Get<math::Vector3d>("servo1_anchor").first;
        
        std::string pitch_joint_name = _sdf->Get<std::string>("pitch_joint", "gimbal_pitch_joint").first;
        std::string yaw_joint_name = _sdf->Get<std::string>("yaw_joint", "gimbal_yaw_joint").first;
        std::string engine_link_name = _sdf->Get<std::string>("engine_link", "engine_link").first;
        
        this->pitch_joint_ = this->model_.JointByName(_ecm, pitch_joint_name);
        this->yaw_joint_ = this->model_.JointByName(_ecm, yaw_joint_name);
        this->engine_link_ = this->model_.LinkByName(_ecm, engine_link_name);

        if (this->pitch_joint_ == kNullEntity || this->yaw_joint_ == kNullEntity || this->engine_link_ == kNullEntity)
        {
            gzerr << "Could not find all required joints/links." << std::endl;
            return;
        }
        
        // --- Initialize servo lengths to neutral (TODO: get from SDF) ---
        // We'll get the real lengths from kinematics in a bit
        this->servo1_current_len_ = 0.1; // TODO: Replace with neutral length
        this->servo2_current_len_ = 0.1; // TODO: Replace with neutral length
        this->servo1_target_len_ = this->servo1_current_len_;
        this->servo2_target_len_ = this->servo2_current_len_;


        // --- Setup Gazebo Transport Subscriber ---
        std::string topic_name = _sdf->Get<std::string>("topic", "/tvc_servo_command").first;
        this->gz_node_.Subscribe(topic_name, &TVCPlugin::OnServoCommand, this);
        
        gzmsg << "TVCPlugin configured for model: " << this->model_.Name(_ecm)
              << " on topic: " << topic_name << std::endl;
    }

    // ISystemPreUpdate
    virtual void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override
    {
        // Don't run if simulation is paused or physics isn't on
        if (_info.paused || !_info.simTime)
        {
            return;
        }

        double dt = std::chrono::duration<double>(_info.dt).count();

        // --- 1. Apply Actuator Dynamics (Slew Rate) ---
        // This is a simple rate limiter
        double max_change = this->servo_max_speed_ * dt;
        
        double s1_error = this->servo1_target_len_ - this->servo1_current_len_;
        this->servo1_current_len_ += std::clamp(s1_error, -max_change, max_change);
        
        double s2_error = this->servo2_target_len_ - this->servo2_current_len_;
        this->servo2_current_len_ += std::clamp(s2_error, -max_change, max_change);

        // TODO: Apply 1st or 2nd order lag filter if needed

        // --- 2. Calculate Kinematics (Servo Lengths -> Gimbal Angles) ---
        // This is the part that needs your specific geometry.
        // It will be a set of trigonometric equations.
        
        // ********** PSEUDOCODE **********
        // double pitch_angle = CalculatePitch(this->servo1_current_len_, this->servo2_current_len_);
        // double yaw_angle = CalculateYaw(this->servo1_current_len_, this->servo2_current_len_);
        // ********************************
        
        // TODO: Replace this with real kinematics
        // For testing, we'll just pass through a simple mapping
        // Let's pretend servo 1 is pitch, servo 2 is yaw
        double pitch_angle = (this->servo1_current_len_ - 0.1) * 1.57; // 0.1 = neutral, 1.57 rad/m
        double yaw_angle   = (this->servo2_current_len_ - 0.1) * 1.57;


        // --- 3. Set Joint Positions ---
        // We force the joints to the kinematically-calculated position.
        // We're not using a PID, we are *setting* the state.
        _ecm.SetComponentData<components::JointPosition>(this->pitch_joint_, {pitch_angle});
        _ecm.SetComponentData<components::JointPosition>(this->yaw_joint_, {yaw_angle});
        
        // --- 4. Apply Thrust Force ---
        // The force is applied along the engine link's local X-axis.
        // Gazebo's physics engine will handle resolving this into forces
        // and torques on the base_link.
        Link engineLink(this->engine_link_);
        engineLink.AddRelativeForce(_ecm, math::Vector3d(this->thrust_, 0, 0));
    }

    // Callback for Gazebo Transport topic
    void OnServoCommand(const msgs::Double_V &_msg)
    {
        if (_msg.data_size() < 2)
        {
            gzerr << "Received servo command with < 2 data elements." << std::endl;
            return;
        }
        
        // TODO: Add min/max length clamping
        this->servo1_target_len_ = _msg.data(0);
        this->servo2_target_len_ = _msg.data(1);
    }

private:
    Model model_{kNullEntity};
    Entity pitch_joint_{kNullEntity};
    Entity yaw_joint_{kNullEntity};
    Entity engine_link_{kNullEntity};
    
    transport::Node gz_node_;
    
    // --- Parameters ---
    double thrust_;
    double servo_max_speed_; // m/s
    
    // --- State ---
    double servo1_current_len_; // The "real" simulated length
    double servo2_current_len_;
    double servo1_target_len_;  // The "commanded" length from Python
    double servo2_target_len_;
    
    // TODO: Add kinematic geometry variables (anchor points, etc.)
};

// Register the plugin
GZ_PLUGIN_REGISTER_CLASS(
    TVCPlugin
)

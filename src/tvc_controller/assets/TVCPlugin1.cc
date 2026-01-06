#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h> // Using Double_V for a 2-vector of servo angles (roll, pitch)

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
        servo_roll_current_angle_(0.0),
        servo_pitch_current_angle_(0.0),
        servo_roll_target_angle_(0.0),
        servo_pitch_target_angle_(0.0)
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
        
        // This is the realistic slew rate of your servos in rad/s.
        // The MightyZap 12LF (4.2mm/s) at a 5cm moment arm is ~0.084 rad/s.
        // Your SDF joint limit is 1.75 rad/s.
        // Use a realistic value here!
        this->servo_max_angular_speed_ = _sdf->Get<double>("servo_max_angular_speed", 0.084).first; 
        
        std::string roll_joint_name = _sdf->Get<std::string>("roll_joint", "servo0_roll_joint").first;
        std::string pitch_joint_name = _sdf->Get<std::string>("pitch_joint", "servo1_pitch_joint").first;
        
        this->roll_joint_ = this->model_.JointByName(_ecm, roll_joint_name);
        this->pitch_joint_ = this->model_.JointByName(_ecm, pitch_joint_name);

        if (this->roll_joint_ == kNullEntity || this->pitch_joint_ == kNullEntity)
        {
            gzerr << "Could not find all required joints (roll_joint, pitch_joint)." << std::endl;
            return;
        }
        
        // --- Initialize servo angles to neutral (0) ---
        this->servo_roll_current_angle_ = 0.0;
        this->servo_pitch_current_angle_ = 0.0;
        this->servo_roll_target_angle_ = 0.0;
        this->servo_pitch_target_angle_ = 0.0;


        // --- Setup Gazebo Transport Subscriber ---
        // This topic will receive the target *angles* (in radians)
        std::string topic_name = _sdf->Get<std::string>("topic", "/tvc_angle_command").first;
        this->gz_node_.Subscribe(topic_name, &TVCPlugin::OnServoAngleCommand, this);
        
        gzmsg << "TVCPlugin configured for model: " << this->model_.Name(_ecm)
              << " on topic: " << topic_name
              << " with max angular speed: " << this->servo_max_angular_speed_ << " rad/s"
              << std::endl;
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
        // This is a simple rate limiter on the *angle* command
        double max_angle_change = this->servo_max_angular_speed_ * dt;
        
        double roll_error = this->servo_roll_target_angle_ - this->servo_roll_current_angle_;
        this->servo_roll_current_angle_ += std::clamp(roll_error, -max_angle_change, max_angle_change);
        
        double pitch_error = this->servo_pitch_target_angle_ - this->servo_pitch_current_angle_;
        this->servo_pitch_current_angle_ += std::clamp(pitch_error, -max_angle_change, max_angle_change);

        // --- 2. Set Joint Positions ---
        // We force the joints to the dynamically-simulated position.
        _ecm.SetComponentData<components::JointPosition>(this->roll_joint_, {this->servo_roll_current_angle_});
        _ecm.SetComponentData<components::JointPosition>(this->pitch_joint_, {this->servo_pitch_current_angle_});
        
        // --- 3. Apply Thrust Force ---
        // NO LONGER NEEDED. Your gz-sim-multicopter-motor-model-system handles this.
        // The thrust is correctly applied to the gimbaled links.
    }

    // Callback for Gazebo Transport topic
    void OnServoAngleCommand(const msgs::Double_V &_msg)
    {
        if (_msg.data_size() < 2)
        {
            gzerr << "Received servo command with < 2 data elements." << std::endl;
            return;
        }
        
        // Data 0: Target Roll Angle (for servo0_roll_joint)
        // Data 1: Target Pitch Angle (for servo1_pitch_joint)
        // TODO: Add min/max angle clamping if desired (e.g., from SDF joint limits)
        this->servo_roll_target_angle_ = _msg.data(0);
        this->servo_pitch_target_angle_ = _msg.data(1);
    }

private:
    Model model_{kNullEntity};
    Entity roll_joint_{kNullEntity};
    Entity pitch_joint_{kNullEntity};
    
    transport::Node gz_node_;
    
    // --- Parameters ---
    double servo_max_angular_speed_; // rad/s
    
    // --- State ---
    double servo_roll_current_angle_; // The "real" simulated angle
    double servo_pitch_current_angle_;
    double servo_roll_target_angle_;  // The "commanded" angle from Python
    double servo_pitch_target_angle_;
};

// Register the plugin
GZ_PLUGIN_REGISTER_CLASS(
    TVCPlugin
)


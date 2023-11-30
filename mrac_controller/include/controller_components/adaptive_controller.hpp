#include "controller_components/controller.hpp"
#include "controller_components/reference_model.hpp"
#include "controller_components/adaptive_law.hpp"

class AdaptiveController: public Controller
{
protected:
    ReferenceModel model;
    AdaptiveLaw law;

    void (*phi_functions) (VectorXd&, const VectorXd);
    double last_zpos;
    double dt;
    
public:
    AdaptiveController(void (*_phi_functions) (VectorXd&, const VectorXd));
    ~AdaptiveController();
    
    virtual void reset();
    virtual void initialize(const PID_Config_t _pid_config_x,
                    const PID_Config_t _pid_config_y,
                    const PID_Config_t _pid_config_z, 
                    const ReferenceModel_Config_t _model_config,
                    const AdaptiveLaw_Config_t _law_config, 
                    const double _dt);
    virtual void compute_outputs(geometry_msgs::msg::Twist* _cmd);

    virtual void get_reference_states(VectorXd &_states);
    virtual void get_adaptive_gains(MatrixXd &gains);
};

#include <iostream>
#include <vector>
#include <fstream>

#include <glm/glm.hpp>

#include <dlfdm/defines.h>
#include <dlfdm/fdmsolver.h>
#include <dlfdm/aerodynamicsmodel.h>

inline dlfdm::AircraftParameters LoadJetTrainerModel() {
    dlfdm::AircraftParameters p;

    // Mass properties
    p.mass = 1815.0f;           // [kg]
    p.Ixx = 1084.6f;            // [kg·m^2]
    p.Iyy = 6507.9f;            // [kg·m^2]
    p.Izz = 7050.2f;            // [kg·m^2]
    p.Ixz = 271.16f;            // [kg·m^2]

    // Aerodynamic reference
    p.wingArea = 12.63f;        // [m^2]
    p.wingChord = 1.64f;        // [m]
    p.wingSpan = 8.01f;         // [m]

    // Propulsion
    p.maxThrust = 11120.0f;     // [N]

    // Aerodynamic coefficients
    p.CL0 = 0.15f;              // [-]
    p.CLa = 5.5f;               // [1/rad]
    p.CL_delta_e = 0.38f;       // [1/rad]
    p.CD0 = 0.0205f;            // [-]
    p.CDa = 0.12f;              // [1/rad]
    p.Cm0 = -0.08f;             // [-]
    p.Cma = -0.24f;             // [1/rad]
    p.Cm_q = -15.7f;            // [1/rad]
    p.CY_beta = -1.0f;          // [1/rad]
    p.CY_r = 0.61f;             // [1/rad]
    p.CY_delta_r = 0.028f;      // [1/rad]
    p.Cl_beta = -0.11f;         // [1/rad]
    p.Cl_p = -0.39f;            // [1/rad]
    p.Cl_r = 0.28f;             // [1/rad]
    p.Cn_beta = 0.17f;          // [1/rad]
    p.Cn_p = 0.09f;             // [1/rad]
    p.Cn_r = -0.26f;            // [1/rad]

    // Control effectiveness
    p.Cm_delta_e = -0.88f;      // [1/rad]
    p.Cl_delta_a = 0.10f;       // [1/rad]
    p.Cn_delta_r = -0.12f;      // [1/rad]

    // Min-max surface deflections
    p.min_elevator = glm::radians(-15.0f);  // [rad]
    p.max_elevator = glm::radians(20.0f);   // [rad]

    p.min_aileron = glm::radians(-20.0f);   // [rad]
    p.max_aileron = glm::radians(20.0f);    // [rad]

    p.max_rudder = glm::radians(20.0f);     // [rad]

    return p;
}

void LoadPhugoidInput(std::vector<glm::vec2>& control_inputs,
                      const float& begin_time,
                      const float& neutral_input)
{
        glm::vec2 elevator_input;
        constexpr float kInputDuration = 0.15f;     // [seg]

        // Doblete de elevador para excitar modo Fugoide
        elevator_input = glm::vec2(begin_time,glm::radians(-10.0f));
        control_inputs.push_back(elevator_input);
        elevator_input = glm::vec2(begin_time + kInputDuration ,glm::radians(10.0f));
        control_inputs.push_back(elevator_input);
        elevator_input = glm::vec2(begin_time + 2 * kInputDuration, neutral_input);
        control_inputs.push_back(elevator_input);
}

void LoadAileronInput(std::vector<glm::vec2>& control_inputs,
                      const float& begin_time,
                      const float& neutral_input)
{
        glm::vec2 aileron_input;
        constexpr float kInputDuration = 0.15f;     // [seg]

        // Deflexión de 5 deg y regreso a posición neutra
        aileron_input = glm::vec2(begin_time,glm::radians(5.0f));
        control_inputs.push_back(aileron_input);
        aileron_input = glm::vec2(begin_time + kInputDuration, neutral_input);
        control_inputs.push_back(aileron_input);
}

void LoadRudderInput(std::vector<glm::vec2>& control_inputs,
                     const float& begin_time,
                     const float& neutral_input)
{
        glm::vec2 rudder_input;
        constexpr float kInputDuration = 0.25f;     // [seg]

        // Doblete de timón de dirección
        rudder_input = glm::vec2(begin_time,glm::radians(-10.0f));
        control_inputs.push_back(rudder_input);
        rudder_input = glm::vec2(begin_time + kInputDuration ,glm::radians(10.0f));
        control_inputs.push_back(rudder_input);
        rudder_input = glm::vec2(begin_time + 2 * kInputDuration, neutral_input);
        control_inputs.push_back(rudder_input);
}

int main()
{
    dlfdm::AircraftParameters airmacci_s211 = LoadJetTrainerModel();

    dlfdm::FDMSolver fdm(airmacci_s211);
    dlfdm::AircraftState init_state;        // Initial conditions for integration

    // -------------------------------------------------------------------------
    // Trim conditions
    init_state.intertial_position = glm::vec3(0.0f,0.0f,5000.0f);     // [m]
    init_state.boby_velocity = glm::vec3(150.0f,0.0f,-0.3667f);       // [m/s]
    init_state.body_omega = glm::vec3(0.0f,0.0f,0.0f);
    init_state.theta = 0.0f;
    init_state.phi = 0.0f;
    init_state.psi = 0.0f;

    dlfdm::ControlInputs controls;
    controls.elevator = -0.09024f;  // [rad]
    controls.aileron = 0.0f;
    controls.rudder = 0.0f;
    controls.throttle = 0.3119f;    // [%]
    // -------------------------------------------------------------------------

    std::cout << "DLFDM" << std::endl;
    std::cout << "----------------------" << std::endl;

    // Load a control input for testing the fdm
    typedef enum ControlsId {kNone = 0, kThrothle, kElevator, kAileron, kRudder } ControlsId;

    std::vector<glm::vec2> control_inputs;
    control_inputs.clear();

    ControlsId use_control_input = ControlsId::kNone;

    switch (use_control_input) {
    case kNone:
        break;
    case kThrothle:
        break;
    case kElevator:
        LoadPhugoidInput(control_inputs,2.5f,controls.elevator);
        break;
    case kAileron:
        LoadAileronInput(control_inputs,2.5f,0.0f);
        break;
    case kRudder:
        LoadRudderInput(control_inputs,2.5f,0.0f);
        break;
    }

    // Total sim time
    const float sim_time = 1.0f * 60.0f;
    const unsigned long steps = static_cast<unsigned long>(sim_time * 120.0f);

    // Load initial conditions (trim) for the solver
    fdm.setState(init_state);

    // Prepare controls input for the loop execution
    size_t current_input = 0;
    glm::vec2 control_input = glm::vec2(0.0f,0.0f);

    // Process inputs if any
    if( control_inputs.size() > 0){
        control_input = control_inputs.at(current_input++);
    }

    // Logging file
    std::fstream logging_file;
    logging_file.open("salida.csv",std::ios_base::out);

    if( !logging_file.is_open() ){
        std::cerr << "Failed to open logging file!" << std::endl;
        return -1;
    }

    // Do some logging
    fdm.log_titles(logging_file);
    fdm.log_state(logging_file);

    std::cout << " Sim time: " << sim_time << " seg" << std::endl;
    std::cout << " Beggining fdm execution ..." << std::endl;

    // Main FDM update
    for (unsigned long i = 0 ; i < steps ; i++) {

        // Update controls inputs
        if( control_inputs.size() > 0 ){
            if( fdm.get_sim_time() > control_input.x ){
                switch (use_control_input) {
                case kNone:
                    break;
                case kThrothle:
                    controls.throttle = control_input.y;
                    break;
                case kElevator:
                    controls.elevator = control_input.y;
                    break;
                case kAileron:
                    controls.aileron = control_input.y;
                    break;
                case kRudder:
                    controls.rudder = control_input.y;
                    break;
                }

                // read next input to be processed
                if( current_input < control_inputs.size() ){
                    control_input = control_inputs.at(current_input++);
                }
            }
        }

        fdm.update(controls);

        // Do data logging
        fdm.log_state(logging_file);
    }

    std::cout << " Done!" << std::endl;

    logging_file.close();

    return 0;
}

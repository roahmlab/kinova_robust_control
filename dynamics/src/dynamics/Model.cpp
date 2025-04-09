#include "dynamics/Model.hpp"

namespace KinovaRobustControl
{
namespace Model
{

int convertPinocchioJointType(const std::string& jtype) {
    if (jtype.find('R') != std::string::npos) {
        if (jtype.find('X') != std::string::npos) {
            return 1;
        }
        else if (jtype.find('Y') != std::string::npos) {
            return 2;
        }
        else if (jtype.find('Z') != std::string::npos) {
            return 3;
        }
        else if (jtype.find('U') != std::string::npos) {
            // This is specific to the digit robot
            // There are 4 joints that have "<axis xyz="0 0 -1"/>"
            // But they can not be identified by urdf parser so we manually set them to be of type -3
            return -3;
        }
        else {
            throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
        }
    }
    else if (jtype.find('P') != std::string::npos) {
        if (jtype.find('X') != std::string::npos) {
            return 4;
        }
        else if (jtype.find('Y') != std::string::npos) {
            return 5;
        }
        else if (jtype.find('Z') != std::string::npos) {
            return 6;
        }
        else {
            throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
        }
    }
    else {
        throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
    }

    return 0;
}

model::model(const std::string& urdf_filename,  
             const double phi_eps_input) {
    if (phi_eps_input < 0) {
        throw std::invalid_argument("phi_eps_input should always be non-negative!");
    }

    pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);
    data_pinocchio = pinocchio::Data(model_pinocchio);

    NB = model_pinocchio.nv; // number of joints

    offset = VecX::Zero(NB);
    friction = VecX::Zero(NB);
    friction_eps = VecX::Constant(NB, phi_eps_input);
    damping = VecX::Zero(NB);
    damping_eps = VecX::Constant(NB, phi_eps_input);
    transmissionInertia = VecX::Zero(NB);
    transmissionInertia_eps = VecX::Constant(NB, phi_eps_input);

    jtype.resize(NB);
    Xtree.resize(NB);
    parent.resize(NB);
    phi = VecX::Zero(10 * NB);
    phi_eps = VecX::Constant(10 * NB, phi_eps_input);
    phi_lb.resize(10 * NB);
    phi_ub.resize(10 * NB);

    readRobotDynamicsFromPinocchio();
}

model::model(
    const std::string& urdf_filename,
    const VecX& friction_input,
    const VecX& friction_eps_input,
    const VecX& damping_input,
    const VecX& damping_eps_input,
    const VecX& transmissionInertia_input,
    const VecX& transmissionInertia_eps_input,
    const VecX& offset_input,
    const VecX& mass_eps_input,
    const VecX& com_eps_input,
    const VecX& inertia_eps_input) :
    friction(friction_input),
    friction_eps(friction_eps_input),
    damping(damping_input),
    damping_eps(damping_eps_input),
    transmissionInertia(transmissionInertia_input),
    transmissionInertia_eps(transmissionInertia_eps_input),
    offset(offset_input) {
    pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);
    data_pinocchio = pinocchio::Data(model_pinocchio);

    NB = model_pinocchio.nv; // number of joints

    if (friction_input.size() != NB) {
        throw std::invalid_argument("friction_input.size() != NB");
    }
    if (friction_eps_input.size() != NB) {
        throw std::invalid_argument("friction_eps_input.size() != NB");
    }
    if (damping_input.size() != NB) {
        throw std::invalid_argument("damping_input.size() != NB");
    }
    if (damping_eps_input.size() != NB) {
        throw std::invalid_argument("damping_eps_input.size() != NB");
    }
    if (transmissionInertia_input.size() != NB) {
        throw std::invalid_argument("transmissionInertia_input.size() != NB");
    }
    if (transmissionInertia_eps_input.size() != NB) {
        throw std::invalid_argument("transmissionInertia_eps_input.size() != NB");
    }
    if (offset_input.size() != NB) {
        throw std::invalid_argument("offset_input.size() != NB");
    }
    if (mass_eps_input.size() != NB) {
        throw std::invalid_argument("mass_eps_input.size() != NB");
    }
    if (com_eps_input.size() != NB) {
        throw std::invalid_argument("com_eps_input.size() != NB");
    }
    if (inertia_eps_input.size() != NB) {
        throw std::invalid_argument("inertia_eps_input.size() != NB");
    }

    jtype.resize(NB);
    Xtree.resize(NB);
    parent.resize(NB);
    phi = VecX::Zero(10 * NB);
    phi_eps = VecX::Zero(10 * NB);
    phi_lb.resize(10 * NB);
    phi_ub.resize(10 * NB);

    for (int i = 0; i < NB; i++) {
        if (inertia_eps_input(i) < 0) {
            throw std::invalid_argument("inertia_eps_input should always be non-negative!");
        }
        if (com_eps_input(i) < 0) {
            throw std::invalid_argument("com_eps_input should always be non-negative!");
        }
        if (mass_eps_input(i) < 0) {
            throw std::invalid_argument("mass_eps_input should always be non-negative!");
        }
        
        phi_eps(10 * i + 0) = mass_eps_input(i);
        phi_eps.segment(10 * i + 1, 3).setConstant(com_eps_input(i));
        phi_eps.segment(10 * i + 4, 6).setConstant(inertia_eps_input(i));
    }

    readRobotDynamicsFromPinocchio();
}

// model::model(const std::string& urdf_filename,
//              const std::string& config_filename) {
//     pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);

//     NB = model_pinocchio.nv; // number of joints

//     offset = VecX::Zero(NB);
//     friction = VecX::Zero(NB);
//     friction_eps = VecX::Zero(NB);
//     damping = VecX::Zero(NB);
//     damping_eps = VecX::Zero(NB);
//     transmissionInertia = VecX::Zero(NB);
//     transmissionInertia_eps = VecX::Zero(NB);

//     jtype.resize(NB);
//     Xtree.resize(NB);
//     parent.resize(NB);
//     phi.resize(10 * NB);
//     phi_eps = VecX::Zero(10 * NB);

//     readRobotDynamicsFromPinocchio();

//     YAML::Node config;
    
//     try {
//         config = YAML::LoadFile(config_filename);
//     }
//     catch (const std::exception& e) {
//         throw std::runtime_error("Failed to load the YAML file." + std::string(e.what()));
//     }

//     if (config.size() < 1 || !config.begin()->second.IsMap()) {
//         throw std::runtime_error("Invalid YAML format. Expected a map as the first node.");
//     }

//     const std::string& robot_name = config.begin()->first.as<std::string>();

//     std::cout << "Robot name: " << robot_name << std::endl;

//     const YAML::Node& robot_config = config[robot_name];

//     for (const auto& entry : robot_config) {
//         const std::string& joint_name = entry.first.as<std::string>();
//         const YAML::Node& joint_properties = entry.second;

//         int joint_id = model_pinocchio.getJointId(joint_name);

//         if (joint_id == -1) {
//             throw std::runtime_error("Joint " + joint_name + " not found in the URDF model.");
//         }

//         joint_id = joint_id - 1; // pinocchio joint id starts from 1

//         if (joint_properties["motor_dynamics"]) {
//             const YAML::Node& motor_dynamics = joint_properties["motor_dynamics"];

//             friction[joint_id] = motor_dynamics["friction"].as<double>();
//             friction_eps[joint_id] = motor_dynamics["friction_eps"].as<double>();
//             damping[joint_id] = motor_dynamics["damping"].as<double>();
//             damping_eps[joint_id] = motor_dynamics["damping_eps"].as<double>();
//             transmissionInertia[joint_id] = motor_dynamics["transmissionInertia"].as<double>();
//             transmissionInertia_eps[joint_id] = motor_dynamics["transmissionInertia_eps"].as<double>();
//             offset[joint_id] = motor_dynamics["offset"].as<double>();
//         }
//         else {
//             throw std::runtime_error("Motor dynamics parameters not found for joint " + joint_name);
//         }

//         if (joint_properties["mass_eps"]) {
//             phi_eps(10 * joint_id + 9) = joint_properties["mass_eps"].as<double>();
//         }
//         else {
//             throw std::runtime_error("mass_eps (mass uncertainty) not found for joint " + joint_name);
//         }

//         if (joint_properties["com_eps"]) {
//             phi_eps.segment(10 * joint_id + 6, 3).setConstant(joint_properties["com_eps"].as<double>());
//         }
//         else {
//             throw std::runtime_error("com_eps (center of mass uncertainty) not found for joint " + joint_name);
//         }

//         if (joint_properties["inertia_eps"]) {
//             phi_eps.segment(10 * joint_id + 0, 6).setConstant(joint_properties["inertia_eps"].as<double>());
//         }
//         else {
//             throw std::runtime_error("inertia_eps (moment of inertia uncertainty) not found for joint " + joint_name);
//         }
//     }
// }

void model::readRobotDynamicsFromPinocchio() {
    for (int i = 0; i < NB; i++) {
        const int pinocchio_joint_id = i + 1; // the first joint in pinocchio is the root joint
        try {
            jtype[i] = convertPinocchioJointType(model_pinocchio.joints[pinocchio_joint_id].shortname());
        }
        catch (const std::invalid_argument& e) {
            std::cerr << e.what() << std::endl;
            std::cerr << "joint name: " << model_pinocchio.joints[pinocchio_joint_id].shortname() << std::endl;
            throw;
        }

        parent[i] = model_pinocchio.parents[pinocchio_joint_id] - 1;

        // plux in Roy Featherstone's code (transformation matrix from parent to child)
        Xtree[i] = Spatial::plux(model_pinocchio.jointPlacements[pinocchio_joint_id].rotation().transpose(), 
                                 model_pinocchio.jointPlacements[pinocchio_joint_id].translation());
        
        // mcI in Roy Featherstone's code (parallel axis theorem)
        phi.segment(10 * i, 10) = model_pinocchio.inertias[pinocchio_joint_id].toDynamicParameters();

        
        for (int j = 0; j < 10; j++) {
            if (phi(10 * i + j) < 0) {
                phi_lb(10 * i + j) = phi(10 * i + j) * (1 + phi_eps(10 * i + j));
                phi_ub(10 * i + j) = phi(10 * i + j) * (1 - phi_eps(10 * i + j));
            }
            else {
                phi_lb(10 * i + j) = phi(10 * i + j) * (1 - phi_eps(10 * i + j));
                phi_ub(10 * i + j) = phi(10 * i + j) * (1 + phi_eps(10 * i + j));
            }
        }
    }

    a_grav << model_pinocchio.gravity.angular(),
              model_pinocchio.gravity.linear();
}

void model::changeEndEffectorInertial(const Vec10& new_phi, 
                                      const Vec10& new_phi_lb,
                                      const Vec10& new_phi_ub) {
    for (int i = 0; i < 10; i++) {
        if (new_phi_lb(i) > new_phi(i) || 
            new_phi(i) > new_phi_ub(i)) {
            std::cerr << i << ' ' << new_phi(i) << ' ' << new_phi_lb(i) << ' ' << new_phi_ub(i) << std::endl;
            throw std::invalid_argument("Inconsistent end effector inertial parameter bounds!");
        }
    }

    std::cerr << "Changing end effector inertial parameters" << std::endl;
    std::cerr << "New phi: " << new_phi.transpose() << std::endl;
    std::cerr << "New phi_lb: " << new_phi_lb.transpose() << std::endl;
    std::cerr << "New phi_ub: " << new_phi_ub.transpose() << std::endl;

    phi.tail(10) = new_phi;
    phi_lb.tail(10) = new_phi_lb;
    phi_ub.tail(10) = new_phi_ub;
}

void model::print() {
    std::cout << "NB: " << NB << std::endl;
    std::cout << "a_grav: " << a_grav.transpose() << std::endl;
    std::cout << "jtype: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << jtype[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "parent: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << parent[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Xtree: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << Xtree[i] << std::endl;
    }

    std::cout << "phi: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << phi.segment(10 * i, 10).transpose() << std::endl;
    }

    std::cout << "phi_eps: " << phi_eps.transpose() << std::endl;
    std::cout << "friction: " << friction.transpose() << std::endl;
    std::cout << "friction_eps: " << friction_eps.transpose() << std::endl;
    std::cout << "damping: " << damping.transpose() << std::endl;
    std::cout << "damping_eps: " << damping_eps.transpose() << std::endl;
    std::cout << "transmissionInertia: " << transmissionInertia.transpose() << std::endl;
    std::cout << "transmissionInertia_eps: " << transmissionInertia_eps.transpose() << std::endl;
    std::cout << "offset: " << offset.transpose() << std::endl;
}

} // namespace Model
} // namespace KinovaRobustControl

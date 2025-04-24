// cfunctions.cpp

#include <Eigen\Dense>
#include <iostream>
#include <chrono>

struct wire_dynamics_return_struct // Struct for the return values from wire dynamics
{
    Eigen::Vector<float, 5> wire_forces;
    int converge_iterations;
};

struct wire_dynamics_trajectory_return_struct // Struct for the return values from wire dynamics
{
    Eigen::Vector<float, 5> wire_forces[10000];
    int converge_iterations;
};


// Functions for running in python
extern "C"
{
    wire_dynamics_return_struct wire_dynamics(Eigen::Vector<float, 3> Forces_desired, Eigen::Matrix<float, 3, 5> wire_directions, Eigen::Matrix<float, 3, 5> moment_arms);
    //wire_dynamics_trajectory_return_struct wire_dynamics_trajectory(Eigen::)
};


wire_dynamics_return_struct wire_dynamics(Eigen::Vector<float, 3> Forces_desired, Eigen::Matrix<float, 3, 5> wire_directions, Eigen::Matrix<float, 3, 5> moment_arms)
{
    //auto start = std::chrono::high_resolution_clock::now();
    // Internal parameters
    const float wire_forces_min=0;
    const float wire_forces_max=5000;
    const float F_tolerance = 1;
    const float M_tolerance = 0.001;
    const int max_iterations = 5000;
    float iteration_impact;
    float step_size_Fx;
    float step_size_Fy;  
    float step_size_Mz;  

    // Initialize wire_forces for first iteration
    Eigen::Vector<float, 5> wire_forces = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Initialize converge iterations counter (+1 to distinguish failed convergence)
    int converge_iterations = max_iterations + 1;

    // Compute gradients with respect to wire forces
    const Eigen::Matrix<float, 3, 5> dF_df = wire_directions;
    Eigen::Matrix<float, 3, 5> dM_df;
    Eigen::Vector<float, 3> moment_arm, wire_direction;

    for(int i = 0; i < 5; i++){
        moment_arm = moment_arms.col(i);
        wire_direction = wire_directions.col(i);
        dM_df.col(i) = moment_arm.cross(wire_direction);
    }

    // Declaration of variables for gradient descent
    Eigen::Vector<float, 3> Forces, Moments;
    Eigen::Vector<float, 2> F_error;
    float M_error;
    Eigen::Vector<float, 5> wire_force_step_x, wire_force_step_y, wire_force_step_z, wire_force_step;

    // Run gradient descent
    for(int i = 1; i <= max_iterations; i++)
    {
        // Compute total forces
        Forces = dF_df * wire_forces;

        // Compute total moments
        Moments = dM_df * wire_forces;

        // Compute errors
        F_error = Forces_desired.segment(0, 2) - Forces.segment(0, 2);
        M_error = Forces_desired(2) - Moments(2);

        // Check for convergence
        if(F_error.norm() <= F_tolerance && abs(M_error) <= M_tolerance)
        {
            converge_iterations = i;
            break;
        }

        // Leaning rate parameters
        //iteration_impact = 0.0001 * i; //TODO: If possible remove this to reduce instability (only if fast enough convergence)
        step_size_Fx = 0.2;// + iteration_impact;  // Step size (learning rate) for Fx
        step_size_Fy = 0.2;// + iteration_impact;  // Step size (learning rate) for Fy
        step_size_Mz = 0.8;// + iteration_impact;  // Step size (learning rate) for Mz

        // Compute individual steps
        wire_force_step_x = step_size_Fx * F_error(0) * dF_df.row(0);
        wire_force_step_y = step_size_Fy * F_error(1) * dF_df.row(1);
        wire_force_step_z = step_size_Mz * M_error * dM_df.row(2);
        wire_force_step = wire_force_step_x + wire_force_step_y + wire_force_step_z;

        // Update wire forces (Check if this is correct)
        wire_forces += wire_force_step;

        // Apply constraints to wire forces
        wire_forces = wire_forces.array().min(wire_forces_max).max(wire_forces_min).matrix();
    }
    // Pack result into the return struct
    wire_dynamics_return_struct result;
    result.wire_forces = wire_forces;
    result.converge_iterations = converge_iterations;

    //auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> duration = end - start;
    //std::cout << "Execution time: " << duration.count() << " seconds." << std::endl;
    //std::cout << "Iterations: " << converge_iterations << std::endl;
    //std::cout << "Wire forces: " << wire_forces << std::endl;
    //std::cout << "Forces:\n" << Forces << std::endl;
    //std::cout << "Moments:\n" << Moments << std::endl;
    //std::cout << "Force Errors:\n" << F_error << std::endl;
    //std::cout << "Moment Errors:\n" << M_error << std::endl;
    //std::cout << "X:\n" << wire_force_step_x << std::endl;
    //std::cout << "Y:\n" << wire_force_step_y << std::endl;
    //std::cout << "Z:\n" << wire_force_step_z << std::endl;
    //std::cout << "Total:\n" << wire_force_step << std::endl;
    
    // Return the result
    return result;
}
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <quad_pos_ctrl/trajectorys.h>
#include <quad_pos_ctrl/ctrl_ref.h>
#include <quad_pos_ctrl/SetTakeoffLand.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"test1");
    ros::NodeHandle n;

    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension),middle(dimension),end(dimension);
    
    // Time count
    ros::Time t0 = ros::Time::now();

    // Add constraints to the vertices
    // curve 1 in paper
        //start.makeStartOrEnd(Eigen::Vector3d(0,0,1),derivative_to_optimize);
        //vertices.push_back(start);

        //middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,0,1));
        //vertices.push_back(middle);

        //middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,2,1));
        //vertices.push_back(middle);

        //end.makeStartOrEnd(Eigen::Vector3d(0,2,1),derivative_to_optimize);
        //vertices.push_back(end);

    // curve 2 in paper
        /*start.makeStartOrEnd(Eigen::Vector3d(-1.2,0,2),derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,-0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,-1.0,1.5));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(-1.2,0,2),derivative_to_optimize);
        vertices.push_back(end);*/

    // curve test 
        /*start.makeStartOrEnd(Eigen::Vector3d(0,0,-1.2),derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,-1.5,-1.4));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,-3.0,-1.2));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1,-1.5,-1));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,0,-1.2));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,1.5,-1.4));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,3.0,-1.2));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1,1.5,-1));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(0,0,-1.2),derivative_to_optimize);
        vertices.push_back(end);*/
    
    // straight line
        start.makeStartOrEnd(Eigen::Vector3d(-1,2,-1.0),derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,2,-1.8));
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(0,0,0));
        middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(-3,0,7.0));
        vertices.push_back(middle);

        /*middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(0,0,0));
        middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(0,0,0.0));
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1,2,-1.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,2,-1.5));
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(0,0,0));
        middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(-3,0,7.0));
        vertices.push_back(middle); */

        end.makeStartOrEnd(Eigen::Vector3d(-1,2,-1.0),derivative_to_optimize);
        vertices.push_back(end);
    // curve 3 in paper
        /*start.makeStartOrEnd(Eigen::Vector3d(-1.2,0,2),derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,-0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.2,0,2));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,-0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.2,0,2));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,1.0,2.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.5,-0.2,2.0));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0.2,-1.0,1.5));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-0.2,-1.0,1.5));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(-1.2,0,2),derivative_to_optimize);
        vertices.push_back(end);*/

    //std::cout << vertices << std::endl;

    //compute the segment times
    std::vector<double> segment_times;
    const double v_max = 2.5;
    const double a_max = 3.5;
    const double magic_fabian_constant = 2.5; // A tuning parameter
    segment_times = estimateSegmentTimes(vertices , v_max , a_max , magic_fabian_constant);
    /*for (int i = 0; i < segment_times.size(); i ++)
    {
        std::cout << segment_times[i] << std::endl;
    }*/
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    parameters.print_debug_info = true;
    //N denotes the number of coefficients of the underlying polynomial 
    //N has to be even.
    //If we want the trajectories to be snap-continuous, N needs to be at least 10.
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters,true);
    opt.setupFromVertices(vertices , segment_times , derivative_to_optimize);
    //opt.solveLinear();
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.optimize();

    ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

    //Obtain the polynomial segments
    mav_trajectory_generation::Segment::Vector segments;
    //opt.getSegments(&segments);
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    /*for (int i = 0; i < segments.size(); i ++){
        std::cout << segments[i] << std::endl;
    }*/
    quad_pos_ctrl::trajectorys trajectorys_msg;
    trajectorys_msg.number_seg = segments.size();
    for (int i = 0; i < segments.size(); i++) {
        quad_pos_ctrl::segments segments_msg;
        segments_msg.dimension = dimension;
        segments_msg.time_seg = segments[i].getTime();
        mav_trajectory_generation::Polynomial::Vector temp_poly = segments[i].getPolynomialsRef();
        for (int j = 0; j < dimension; j++) {
            quad_pos_ctrl::factors factors_msg;
            factors_msg.order = N;
            Eigen::VectorXd temp_factors = temp_poly[j].getCoefficients();
            for (int k = 0; k < N; k++) {
                factors_msg.coeffs.push_back(temp_factors[k]);
            }
            segments_msg.factors.push_back(factors_msg);
        }
        trajectorys_msg.segments.push_back(segments_msg);
    }
    //creating Trajectories
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    //evaluating the trajectory at particular instances of time
    // Single sample:
    double sampling_time = 2.0;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);
    // Sample range:
    double t_start = 1.0;
    double t_end = 5.0;
    double dt = 0.01;
    std::vector<Eigen::VectorXd> result;
    std::vector<double> sampling_times; // Optional.
    trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
    //std::cout << result.size() << std::endl;
    //std::cout << sampling_times.size() << std::endl;
    //visualizing Trajectories
    visualization_msgs::MarkerArray markers;
    double distance = 1.6 ;// Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    // From Trajectory class:
    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject" , 10);
    //ros::Publisher traj_pub = n.advertise<quad_pos_ctrl::trajectorys>( "traj_ref" , 10);
    ros::ServiceClient takeoff_land_srv = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node5/controller/takeoff_land");
    ros::Publisher ctrl_ref_pub = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node5/controller/ctrl_ref",10);

    quad_pos_ctrl::ctrl_ref traject_ctrl_ref;
    ros::Rate sleep_rate1(10);
    for (int i = 0; i < 10; i++) {
         ros::spinOnce();
         vis_pub.publish(markers);
         sleep_rate1.sleep();
    }
    char ask;
    std::cout << "total flight time of traject is " << trajectory.getMaxTime() << "s" << std::endl;
    std::cout << "new traject has generated, ready to send. press [Y/N] to countinue?" << std::endl;
    std::cin >> ask;
    if ((ask == 'Y') or (ask == 'y')) {
        std::cout << "sending" << std::endl;
        std::cout << "will fly after 5 s" << std::endl;
    } else {
        return 0;
    }

    quad_pos_ctrl::SetTakeoffLand set_takeoff_land;
    set_takeoff_land.request.takeoff = true;
    set_takeoff_land.request.takeoff_altitude = 0.5f;
    takeoff_land_srv.call(set_takeoff_land);
    ROS_INFO("takeoff test");
    sleep(5);
      // rate 
    ros::Rate sleep_rate(50);
    ros::Time start_time = ros::Time::now() + ros::Duration(5.0);
    while (ros::ok()) {
         ros::spinOnce();
         traject_ctrl_ref.header.stamp = ros::Time::now();

         double _t = (ros::Time::now() - start_time).toSec();
         if (_t > 0.0f && _t < trajectory.getMaxTime()) {
            sampling_time = _t;
            traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED | quad_pos_ctrl::ctrl_ref::VEL_CTRL_VALIED | quad_pos_ctrl::ctrl_ref::ACC_CTRL_VALIED;
         } else if ( _t >= trajectory.getMaxTime()) {
            sampling_time = trajectory.getMaxTime();
            traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
         } else {
            sampling_time = 0.0f;
            traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
         }

        derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        sample = trajectory.evaluate(sampling_time, derivative_order);
        std::cout << "POS: ";
        for (int i = 0; i < sample.size(); i++) {
            std::cout << sample(i) << ',';
            traject_ctrl_ref.pos_ref[i] = sample(i);
        }
        std::cout << std::endl;

        derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
        sample = trajectory.evaluate(sampling_time, derivative_order);
        std::cout << "VEL: ";
        for (int i = 0; i < sample.size(); i++) {
            std::cout << sample(i) << ',';
            traject_ctrl_ref.vel_ref[i] = sample(i);
        }
        std::cout << std::endl;

        derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
        sample = trajectory.evaluate(sampling_time, derivative_order);
        std::cout << "ACC: ";
        for (int i = 0; i < sample.size(); i++) {
            std::cout << sample(i) << ',';
            traject_ctrl_ref.acc_ref[i] = sample(i);
        }
        std::cout << std::endl;

        traject_ctrl_ref.yaw_ref = -1.57f;
        
        ctrl_ref_pub.publish(traject_ctrl_ref);
         //vis_pub.publish(markers);
         //traj_pub.publish(trajectorys_msg);

        if ( _t >= (trajectory.getMaxTime() + 5.0f) ) {

            set_takeoff_land.request.takeoff = false;
            takeoff_land_srv.call(set_takeoff_land);
            ROS_INFO("land test");

            break;
        }

         sleep_rate.sleep();
    }
    return 0;
}
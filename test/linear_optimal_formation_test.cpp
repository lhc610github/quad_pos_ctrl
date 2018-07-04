#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <quad_pos_ctrl/trajectorys.h>
#include <quad_pos_ctrl/ctrl_ref.h>
#include <quad_pos_ctrl/SetTakeoffLand.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"test1");
    ros::NodeHandle n;

    mav_trajectory_generation::Vertex::Vector vertices[5];
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension),middle(dimension),end(dimension);

    // uav 1#
    start.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    start.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    start.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    end.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    end.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    end.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
    start.makeStartOrEnd(Eigen::Vector3d(0,3.0,-1.3),derivative_to_optimize);
    vertices[0].push_back(start);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,3.2,-1.3));
    middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(0.5,0,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(0,0,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::JERK,Eigen::Vector3d(0,0,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::SNAP,Eigen::Vector3d(0,0,0));
    vertices[0].push_back(middle);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::JERK);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::SNAP);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,3.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,2.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,1.5,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,-1.5,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1.3,-3.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,-3.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.3,-3.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.3,-1.5,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.3,0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.3,1.5,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.3,2.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(-1.5,3.0,-1.3));
    vertices[0].push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(0,3.2,-1.3));
    middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(0,-0.5,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(0,0,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::JERK,Eigen::Vector3d(0,0,0));
    middle.addConstraint(mav_trajectory_generation::derivative_order::SNAP,Eigen::Vector3d(0,0,0));
    vertices[0].push_back(middle);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::JERK);
    middle.removeConstraint(mav_trajectory_generation::derivative_order::SNAP);
    end.makeStartOrEnd(Eigen::Vector3d(0,-3.0,-1.3),derivative_to_optimize);
    vertices[0].push_back(end);
    
    int temp_length = vertices[0].size();
    std::cout << temp_length << std::endl;
    // uav 2#
    vertices[1] = vertices[0];
    start.makeStartOrEnd(Eigen::Vector3d(0,1.5,-1.3),derivative_to_optimize);
    vertices[1][0] = start;
    end.makeStartOrEnd(Eigen::Vector3d(0,-1.5,-1.3),derivative_to_optimize);
    vertices[1][temp_length-1] = end;
    // uav 3#
    vertices[2] = vertices[0];
    start.makeStartOrEnd(Eigen::Vector3d(0,0.0,-1.3),derivative_to_optimize);
    vertices[2][0] = start;
    end.makeStartOrEnd(Eigen::Vector3d(0,0.0,-1.3),derivative_to_optimize);
    vertices[2][temp_length-1] = end;
    // uav 4#
    vertices[3] = vertices[0];
    start.makeStartOrEnd(Eigen::Vector3d(0,-1.5,-1.3),derivative_to_optimize);
    vertices[3][0] = start;
    end.makeStartOrEnd(Eigen::Vector3d(0,1.5,-1.3),derivative_to_optimize);
    vertices[3][temp_length-1] = end;
    // uav 5#
    vertices[4] = vertices[0];
    start.makeStartOrEnd(Eigen::Vector3d(0,-3.0,-1.3),derivative_to_optimize);
    vertices[4][0] = start;
    end.makeStartOrEnd(Eigen::Vector3d(0,3.0,-1.3),derivative_to_optimize);
    vertices[4][temp_length-1] = end;
    

    // Time count
    ros::Time t0 = ros::Time::now();


    //compute the segment times
    std::vector<double> segment_times[5];
    const double v_max = 2.5;
    const double a_max = 2.5;
    const double magic_fabian_constant = 2.5; // A tuning parameter
    // uav 1#
    segment_times[0] = estimateSegmentTimes(vertices[0] , v_max , a_max , magic_fabian_constant);
    temp_length = segment_times[0].size();
    std::cout << temp_length << std::endl;
    segment_times[0][0] = 2.5f;
    segment_times[0][temp_length-1] = 12.5f;
    // uav 2#
    segment_times[1] = segment_times[0];
    segment_times[1][0] = 5.0f;
    segment_times[1][temp_length-1] = 10.0f;
    // uav 3#
    segment_times[2] = segment_times[0];
    segment_times[2][0] = 7.5f;
    segment_times[2][temp_length-1] = 7.5f;
    // uav 4#
    segment_times[3] = segment_times[0];
    segment_times[3][0] = 10.0f;
    segment_times[3][temp_length-1] = 5.0f;
    // uav 5#
    segment_times[4] = segment_times[0];
    segment_times[4][0] = 12.5f;
    segment_times[4][temp_length-1] = 2.5f;
    for (int j = 0; j < 5; j ++) {
        for (int i = 0; i < segment_times[j].size(); i ++)
        {
            std::cout << segment_times[j][i] << ' ';
        }
        std::cout << std::endl;
    }
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    parameters.print_debug_info = true;
    
    //Obtain the polynomial segments
    mav_trajectory_generation::Segment::Vector segments[5];
    mav_trajectory_generation::Trajectory trajectory[5];
    for (int i = 0; i < 5; i++) {
        //N denotes the number of coefficients of the underlying polynomial 
        //N has to be even.
        //If we want the trajectories to be snap-continuous, N needs to be at least 10.
        const int N = 10;
        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters,true);
        opt.setupFromVertices(vertices[i] , segment_times[i] , derivative_to_optimize);
        opt.solveLinear();
        //opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
        //opt.optimize();

        //ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

        //opt.getSegments(&segments);
        opt.getPolynomialOptimizationRef().getSegments(&segments[i]);
        //creating Trajectories
        opt.getTrajectory(&trajectory[i]);
        printf("index :%d done\n",i);
    }
    /*for (int i = 0; i < segments.size(); i ++){
        std::cout << segments[i] << std::endl;
    }*/
    /*quad_pos_ctrl::trajectorys trajectorys_msg;
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
    }*/

    //visualizing Trajectories
    visualization_msgs::MarkerArray markers[5];
    for (int i = 0; i < 5; i++ ) {

        //evaluating the trajectory at particular instances of time
        // Single sample:
        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory[i].evaluate(sampling_time, derivative_order);
        // Sample range:
        double t_start = 1.0;
        double t_end = 5.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory[i].evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
        //std::cout << result.size() << std::endl;
        //std::cout << sampling_times.size() << std::endl;
        double distance = 1.6 ;// Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        // From Trajectory class:
        mav_trajectory_generation::drawMavTrajectory(trajectory[i], distance, frame_id, &markers[i]);
    }

    ros::Publisher vis_pub[5];
    vis_pub[0] = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject1" , 10);
    vis_pub[1] = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject2" , 10);
    vis_pub[2] = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject3" , 10);
    vis_pub[3] = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject4" , 10);
    vis_pub[4] = n.advertise<visualization_msgs::MarkerArray>( "trajectory_traject5" , 10);

    ros::Publisher dpos_vis_pub[5];
    dpos_vis_pub[0] = n.advertise<visualization_msgs::Marker>( "dpos1" , 10);
    dpos_vis_pub[1] = n.advertise<visualization_msgs::Marker>( "dpos2" , 10);
    dpos_vis_pub[2] = n.advertise<visualization_msgs::Marker>( "dpos3" , 10);
    dpos_vis_pub[3] = n.advertise<visualization_msgs::Marker>( "dpos4" , 10);
    dpos_vis_pub[4] = n.advertise<visualization_msgs::Marker>( "dpos5" , 10);

    ros::ServiceClient takeoff_land_srv[5];
    takeoff_land_srv[0] = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node1/controller/takeoff_land");
    takeoff_land_srv[1] = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node2/controller/takeoff_land");
    takeoff_land_srv[2] = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node3/controller/takeoff_land");
    takeoff_land_srv[3] = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node4/controller/takeoff_land");
    takeoff_land_srv[4] = n.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node5/controller/takeoff_land");

    ros::Publisher ctrl_ref_pub[5];
    ctrl_ref_pub[0] = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node1/controller/ctrl_ref",10);
    ctrl_ref_pub[1] = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node2/controller/ctrl_ref",10);
    ctrl_ref_pub[2] = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node3/controller/ctrl_ref",10);
    ctrl_ref_pub[3] = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node4/controller/ctrl_ref",10);
    ctrl_ref_pub[4] = n.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node5/controller/ctrl_ref",10);

    /* stage one: vis the traj */
    ros::Rate sleep_rate1(10);
    for (int i = 0; i < 10; i++) {
         ros::spinOnce();
         for (int j = 0; j < 5; j++) {
            vis_pub[j].publish(markers[j]);
         }
         sleep_rate1.sleep();
    }

    char ask;
    std::cout << "new traject has generated, ready to send. press [Y/N] to countinue?" << std::endl;
    std::cin >> ask;
    if ((ask == 'Y') or (ask == 'y')) {
        std::cout << "sending" << std::endl;
        std::cout << "will fly after 5 s" << std::endl;
    } else {
        return 0;
    }


    /* stage two: takeoff the uav */
    quad_pos_ctrl::SetTakeoffLand set_takeoff_land;
    set_takeoff_land.request.takeoff = true;
    set_takeoff_land.request.takeoff_altitude = 0.9f;
    takeoff_land_srv[0].call(set_takeoff_land);
    takeoff_land_srv[1].call(set_takeoff_land);
    takeoff_land_srv[2].call(set_takeoff_land);
    takeoff_land_srv[3].call(set_takeoff_land);
    takeoff_land_srv[4].call(set_takeoff_land);
    ROS_INFO("takeoff test");
    sleep(5);


    ros::Rate sleep_rate(50);
    ros::Time start_time = ros::Time::now() + ros::Duration(5.0);
    quad_pos_ctrl::ctrl_ref traject_ctrl_ref;
    while (ros::ok()) {
        ros::spinOnce();
        traject_ctrl_ref.header.stamp = ros::Time::now();

        for (int i = 0; i < 5;i ++) {
            //vis_pub[i].publish(markers[i]);
            //traject_ctrl_ref.header.stamp = ros::Time::now();
            double sampling_time;

            double _t = (ros::Time::now() - start_time).toSec();
            if (_t > 0.0f && _t < trajectory[i].getMaxTime()) {
                sampling_time = _t;
                traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED | quad_pos_ctrl::ctrl_ref::VEL_CTRL_VALIED | quad_pos_ctrl::ctrl_ref::ACC_CTRL_VALIED;
            } else if ( _t >= trajectory[i].getMaxTime()) {
                sampling_time = trajectory[i].getMaxTime();
                traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
            } else {
                sampling_time = 0.0f;
                traject_ctrl_ref.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
            }
            int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
            Eigen::VectorXd sample = trajectory[i].evaluate(sampling_time, derivative_order);
            //std::cout << sampling_time << std::endl;
            //std::cout << "POS: ";
            for (int j = 0; j < sample.size(); j++) {
                //std::cout << sample(j) << ',';
                traject_ctrl_ref.pos_ref[j] = sample(j);
            }
            Eigen::VectorXd dpos_sample = sample;
            //std::cout << std::endl;

            derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
            sample = trajectory[i].evaluate(sampling_time, derivative_order);
            //std::cout << "VEL: ";
            for (int j = 0; j < sample.size(); j++) {
                //std::cout << sample(j) << ',';
                traject_ctrl_ref.vel_ref[j] = sample(j);
            }
            //std::cout << std::endl;

            derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
            sample = trajectory[i].evaluate(sampling_time, derivative_order);
            //std::cout << "ACC: ";
            for (int j = 0; j < sample.size(); j++) {
                //std::cout << sample(j) << ',';
                traject_ctrl_ref.acc_ref[j] = sample(j);
            }
            //std::cout << std::endl;

            traject_ctrl_ref.yaw_ref = -1.57f;
            
            ctrl_ref_pub[i].publish(traject_ctrl_ref);

            if ( _t >= (trajectory[i].getMaxTime() + 5.0f) ) {

                set_takeoff_land.request.takeoff = false;
                //takeoff_land_srv[i].call(set_takeoff_land);
                takeoff_land_srv[0].call(set_takeoff_land);
                takeoff_land_srv[1].call(set_takeoff_land);
                takeoff_land_srv[2].call(set_takeoff_land);
                takeoff_land_srv[3].call(set_takeoff_land);
                takeoff_land_srv[4].call(set_takeoff_land);
                ROS_INFO("land test");
                //break;
            }


            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "dpos";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = (1.0/5.0)*(float)(5-i);
            marker.color.g = (1.0/5.0)*(float)i;
            marker.color.b = 0.0;
            marker.pose.position.x = dpos_sample(0);
            marker.pose.position.y = dpos_sample(1);
            marker.pose.position.z = dpos_sample(2);
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            dpos_vis_pub[i].publish(marker);
        }
        sleep_rate.sleep();
    }
    return 0;
}
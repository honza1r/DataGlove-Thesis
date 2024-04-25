classdef MadgwickFull < matlab.System
    % untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties (Access = public)
    samplingFrequency;
    gyro_x_corr;
    gyro_y_corr;
    gyro_z_corr;
    end

    properties (DiscreteState)
    qDot1;
    qDot2;
    qDot3;
    qDot4;
    q0;
    q1;
    q2;
    q3;
    ax;
    ay;
    az;
    recipNorm;
    operating_time;
    twoq0mx;
    twoq0my;
    twoq0mz;
    twoq1mx;
    twoq0;
    twoq1;
    twoq2;
    twoq3;
    twoq0q2;
    twoq2q3;
    q0q0;
    q0q1;
    q0q2;
    q0q3;
    q1q1;
    q1q2;
    q1q3;
    q2q2;
    q2q3;
    q3q3;
    s0;
    s1;
    s2
    s3;
    beta;
    mx;
    my;
    mz;
    hx;
    hy;
    twobx;
    twobz;
    fourbx;
    fourbz;
    time_beta;
    end

    % Pre-computed constants
    properties (Access = private)

    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.qDot1 = 0.0;
            obj.qDot2 = 0.0;
            obj.qDot3 = 0.0;
            obj.qDot4 = 0.0;
            obj.q0 = 1.0;
            obj.q1 = 0.0;
            obj.q2 = 0.0;
            obj.q3 = 0.0;
            obj.operating_time = 1/obj.samplingFrequency;
            obj.ax = 0.0;
            obj.ay = 0.0;
            obj.az = 0.0;
            obj.recipNorm = 0.0;
            obj.twoq0mx = 0.0;
            obj.twoq0my = 0.0;
            obj.twoq0mz = 0.0;
            obj.twoq1mx = 0.0;
            obj.twoq0 = 0.0;
            obj.twoq1 = 0.0;
            obj.twoq2 = 0.0;
            obj.twoq3 = 0.0;
            obj.twoq0q2 = 0.0;
            obj.twoq2q3 = 0.0;
            obj.q0q0 = 0.0;
            obj.q0q1 = 0.0;
            obj.q0q2 = 0.0;
            obj.q0q3 = 0.0;
            obj.q1q1 = 0.0;
            obj.q1q2 = 0.0;
            obj.q1q3 = 0.0;
            obj.q2q2 = 0.0;
            obj.q2q3 = 0.0;
            obj.q3q3 = 0.0;
            obj.s0 = 0.0;
            obj.s1 = 0.0;
            obj.s2 = 0.0;
            obj.s3 = 0.0;
            obj.beta = 0.02;%this is the value we tune
            %the higher the value the more reponsive and more closely is
            %the sensor followed, BUT, also the more unstable the
            %readings, especially if roll goes above 45deg(yaw drifts a little but comes back once roll decreses again), 
            %I would not go above 0.2 (depending on the system and
            %requeirements of course)
            %lower values have much smaller prolbem with remaining stable
            %but arent as responsive (though seem responsive enough for
            %most applications I'd say), Id use a value between 0.01 and
            %0.1 in this case
            %I cant see an obvious effect of different frequency (but I
            %only tested 104 and 208Hz)
            %settings that I find work:
            %208hz, +/-8g, 1000dps, beta=0.02
            obj.mx = 0.0;
            obj.my = 0.0;
            obj.mz = 0.0;
            obj.hx = 0.0;
            obj.hy = 0.0;
            obj.twobx = 0.0;
            obj.twobz = 0.0;
            obj.fourbx = 0.0;
            obj.fourbz = 0.0;
            obj.time_beta = 0;
        end

        function [roll,pitch,yaw] = stepImpl(obj,accelerometer,gyroscope, magnetometer)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if obj.time_beta < 5000
                obj.beta = 0.1;
                obj.time_beta = obj.time_beta + 1;
            else 
                obj.beta = 0.01;
            end
            %explanation for this at the end
            obj.ax = accelerometer(1);
            obj.ay = accelerometer(2);
            obj.az = accelerometer(3);
            obj.mx = magnetometer(1);
            obj.my = magnetometer(2);
            obj.mz = magnetometer(3);
            obj.qDot1 = 0.5*(-obj.q1*(gyroscope(1)-obj.gyro_x_corr) - obj.q2*(gyroscope(2)-obj.gyro_y_corr) - obj.q3*(gyroscope(3)-obj.gyro_z_corr));
            obj.qDot2 = 0.5*(obj.q0*(gyroscope(1)-obj.gyro_x_corr) + obj.q2*(gyroscope(3)-obj.gyro_z_corr) - obj.q3*(gyroscope(2)-obj.gyro_y_corr));
            obj.qDot3 = 0.5*(obj.q0*(gyroscope(2)-obj.gyro_y_corr) - obj.q1*(gyroscope(3)-obj.gyro_z_corr) + obj.q3*(gyroscope(1)-obj.gyro_x_corr));
            obj.qDot4 = 0.5*(obj.q0*(gyroscope(3)-obj.gyro_z_corr) + obj.q1*(gyroscope(2)-obj.gyro_y_corr) - obj.q2*(gyroscope(1)-obj.gyro_x_corr));

            if (~((obj.ax == 0.0) && (obj.ay == 0.0) && (obj.az == 0.0)))
                obj.recipNorm = 1/(sqrt(obj.ax^2+obj.ay^2+obj.az^2));
                obj.ax = obj.ax*obj.recipNorm;
                obj.ay = obj.ay*obj.recipNorm;
                obj.az = obj.az*obj.recipNorm;

                obj.recipNorm = 1/(sqrt(obj.mx^2+obj.my^2+obj.mz^2));
                obj.mx = obj.mx*obj.recipNorm;
                obj.my = obj.my*obj.recipNorm;
                obj.mz = obj.mz*obj.recipNorm;
                
                obj.twoq0mx = 2.0*obj.q0*obj.mx;
                obj.twoq0my = 2.0*obj.q0*obj.my;
                obj.twoq0mz = 2.0*obj.q0*obj.mz;
                obj.twoq1mx = 2.0*obj.q1*obj.mx;
                obj.twoq0 = 2.0*obj.q0;
                obj.twoq1 = 2.0*obj.q1;
                obj.twoq2 = 2.0*obj.q2;
                obj.twoq3 = 2.0*obj.q3;
                obj.twoq0q2 = 2.0*obj.q0*obj.q2;
                obj.twoq2q3 = 2.0*obj.q2*obj.q3;
                obj.q0q0 = obj.q0^2;
                obj.q0q1 = obj.q0*obj.q1;
                obj.q0q2 = obj.q0*obj.q2;
                obj.q0q3 = obj.q0*obj.q3;
                obj.q1q1 = obj.q1^2;
                obj.q1q2 = obj.q1*obj.q2;
                obj.q1q3 = obj.q1*obj.q3;
                obj.q2q2 = obj.q2^2;
                obj.q2q3 = obj.q2*obj.q3;
                obj.q3q3 = obj.q3^2;

                obj.hx = obj.mx*obj.q0q0 - obj.twoq0my*obj.q3 + obj.twoq0mz*obj.q2 + obj.mx*obj.q1q1 + obj.twoq1*obj.my*obj.q2 + obj.twoq1*obj.mz*obj.q3 - obj.mx*obj.q2q2 - obj.mx*obj.q3q3;
                obj.hy = obj.twoq0mx*obj.q3 + obj.my*obj.q0q0 - obj.twoq0mz*obj.q1 + obj.twoq1mx*obj.q2 - obj.my*obj.q1q1 + obj.my*obj.q2q2 + obj.twoq2*obj.mz*obj.q3 - obj.my*obj.q3q3;
                
                obj.twobx = sqrt(obj.hx^2 + obj.hy^2);
                obj.twobz = -obj.twoq0mx*obj.q2 + obj.twoq0my*obj.q1 + obj.mz*obj.q0q0 + obj.twoq1mx*obj.q3 - obj.mz*obj.q1q1 + obj.twoq2*obj.my*obj.q3 - obj.mz*obj.q2q2 + obj.mz*obj.q3q3;
                obj.fourbx = 2.0*obj.twobx;
                obj.fourbz = 2.0*obj.twobz;
                
                obj.s0 = -obj.twoq2 * (2.0 * obj.q1q3 - obj.twoq0q2 - obj.ax) + obj.twoq1 * (2.0 * obj.q0q1 + obj.twoq2q3 - obj.ay) - obj.twobz * obj.q2 * (obj.twobx * (0.5 - obj.q2q2 - obj.q3q3) + obj.twobz * (obj.q1q3 - obj.q0q2) - obj.mx) + (-obj.twobx * obj.q3 + obj.twobz * obj.q1) * (obj.twobx * (obj.q1q2 - obj.q0q3) + obj.twobz * (obj.q0q1 + obj.q2q3) - obj.my) + obj.twobx * obj.q2 * (obj.twobx * (obj.q0q2 + obj.q1q3) + obj.twobz * (0.5 - obj.q1q1 - obj.q2q2) - obj.mz);
		        obj.s1 = obj.twoq3 * (2.0 * obj.q1q3 - obj.twoq0q2 - obj.ax) + obj.twoq0 * (2.0 * obj.q0q1 + obj.twoq2q3 - obj.ay) - 4.0 * obj.q1 * (1 - 2.0 * obj.q1q1 - 2.0 * obj.q2q2 - obj.az) + obj.twobz * obj.q3 * (obj.twobx * (0.5 - obj.q2q2 - obj.q3q3) + obj.twobz * (obj.q1q3 - obj.q0q2) - obj.mx) + (obj.twobx * obj.q2 + obj.twobz * obj.q0) * (obj.twobx * (obj.q1q2 - obj.q0q3) + obj.twobz * (obj.q0q1 + obj.q2q3) - obj.my) + (obj.twobx * obj.q3 - obj.fourbz * obj.q1) * (obj.twobx * (obj.q0q2 + obj.q1q3) + obj.twobz * (0.5 - obj.q1q1 - obj.q2q2) - obj.mz);
		        obj.s2 = -obj.twoq0 * (2.0 * obj.q1q3 - obj.twoq0q2 - obj.ax) + obj.twoq3 * (2.0 * obj.q0q1 + obj.twoq2q3 - obj.ay) - 4.0 * obj.q2 * (1 - 2.0 * obj.q1q1 - 2.0 * obj.q2q2 - obj.az) + (-obj.fourbx * obj.q2 - obj.twobz * obj.q0) * (obj.twobx * (0.5 - obj.q2q2 - obj.q3q3) + obj.twobz * (obj.q1q3 - obj.q0q2) - obj.mx) + (obj.twobx * obj.q1 + obj.twobz * obj.q3) * (obj.twobx * (obj.q1q2 - obj.q0q3) + obj.twobz * (obj.q0q1 + obj.q2q3) - obj.my) + (obj.twobx * obj.q0 - obj.fourbz * obj.q2) * (obj.twobx * (obj.q0q2 + obj.q1q3) + obj.twobz * (0.5 - obj.q1q1 - obj.q2q2) - obj.mz);
		        obj.s3 = obj.twoq1 * (2.0 * obj.q1q3 - obj.twoq0q2 - obj.ax) + obj.twoq2 * (2.0 * obj.q0q1 + obj.twoq2q3 - obj.ay) + (-obj.fourbx * obj.q3 + obj.twobz * obj.q1) * (obj.twobx * (0.5 - obj.q2q2 - obj.q3q3) + obj.twobz * (obj.q1q3 - obj.q0q2) - obj.mx) + (-obj.twobx * obj.q0 + obj.twobz * obj.q2) * (obj.twobx * (obj.q1q2 - obj.q0q3) + obj.twobz * (obj.q0q1 + obj.q2q3) - obj.my) + obj.twobx * obj.q1 * (obj.twobx * (obj.q0q2 + obj.q1q3) + obj.twobz * (0.5 - obj.q1q1 - obj.q2q2) - obj.mz);

                obj.recipNorm = 1/sqrt(obj.s0^2 + obj.s1^2 + obj.s2^2 + obj.s3^2);
                obj.s0 = obj.s0*obj.recipNorm;
                obj.s1 = obj.s1*obj.recipNorm;
                obj.s2 = obj.s2*obj.recipNorm;
                obj.s3 = obj.s3*obj.recipNorm;
                
                obj.qDot1 = obj.qDot1-obj.beta*obj.s0;
                obj.qDot2 = obj.qDot2-obj.beta*obj.s1;
                obj.qDot3 = obj.qDot3-obj.beta*obj.s2;
                obj.qDot4 = obj.qDot4-obj.beta*obj.s3;
            end
            
            obj.q0 = obj.q0 + obj.qDot1*obj.operating_time;
            obj.q1 = obj.q1 + obj.qDot2*obj.operating_time;
            obj.q2 = obj.q2 + obj.qDot3*obj.operating_time;
            obj.q3 = obj.q3 + obj.qDot4*obj.operating_time;

            obj.recipNorm = 1/sqrt(obj.q0^2 + obj.q1^2 + obj.q2^2 + obj.q3^2);
            obj.q0 = obj.q0*obj.recipNorm;
            obj.q1 = obj.q1*obj.recipNorm;
            obj.q2 = obj.q2*obj.recipNorm;
            obj.q3 = obj.q3*obj.recipNorm;

            roll = atan2(obj.q0*obj.q1 + obj.q2*obj.q3, 0.5 - obj.q1^2 - obj.q2^2)*180.0/3.14;
            pitch = -asin(-2.0*(obj.q1*obj.q3 - obj.q0*obj.q2))*180.0/3.14+2.5;
            yaw = -atan2(obj.q1*obj.q2 + obj.q0*obj.q3, 0.5 - obj.q2^2 - obj.q3^2)*180.0/3.14-40.0;
            %pitch and yaw have - in front just to inverse directions of
            %rotation, just to make the visualization a little prettire,
            %yaw also has an offset of 35deg, it is worth mentioning that
            %every time the microsontroller is started, the yaw is off and
            %needs to move into its "equilibrium" position, that means that
            %it tracks correclty, but until it moves into its "equilibrium"
            %position it will drift slightly, the higher the beta value the
            %faster it will mvoe into this position, at beta=0.01 it moves
            %very slwoly which measn it takes a while but it also has a
            %tiny drift. experimentation is needed to see what value works
            %best for an aplication, same goes for pitch actaully, maybe
            %even roll, tohugh I didnt notice that explicitely
               %to solve this, I will set beta to 0.1 for the first 5
               %seconds (ish) adn then set it to 0.01, to first make it settle
               %and then get good beta value for tracking


            % these euler angles are kinda weird when you track them as
            % numbers themsevles, but it may be better to just stick to
            % quaternion representation, though that will require properly
            % understanding quaternions
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end

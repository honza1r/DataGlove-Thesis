classdef EKF_Roll_Pitch_Yaw < matlab.System
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
    phi;
    theta;
    psi;
    P;
    Q;
    R;
    time;
    end

    % Pre-computed constants
    properties (Access = private)

    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.phi = 0;
            obj.theta = 0;
            obj.psi = 0;
            obj.P = eye(3)*0.1;
            obj.Q = eye(3)*0.001;
            obj.R = eye(5)*0.011;
            obj.time = 0;
        end

        function [roll,pitch,yaw] = stepImpl(obj,accelerometer, gyroscope, magnetometer)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            g = [-gyroscope(2)+obj.gyro_y_corr;
                 -gyroscope(1)+obj.gyro_x_corr;
                 -gyroscope(3)+obj.gyro_z_corr];
            a = [-accelerometer(2);
                 -accelerometer(1);
                 -accelerometer(3)];
            mag = (a(1)^2+a(2)^2+a(3)^2)^0.5;
            a = a/mag;
            m = [magnetometer(1);
                 magnetometer(2);
                 magnetometer(3)];
            if obj.time < 300
                obj.phi = atan(a(2)/a(3));
                obj.theta = asin(a(1));
                obj.psi = atan2(m(1),m(2));
                obj.time = obj.time + 1;
            else 
            %Predict Start
            phi_dot = g(1) + g(2)*sin(obj.phi)*tan(obj.theta) + g(3)*cos(obj.phi)*tan(obj.theta);
            theta_dot = (g(2)*cos(obj.phi) - g(3)*sin(obj.phi));
            psi_dot = (g(2)*sin(obj.phi)/cos(obj.theta) + g(3)*cos(obj.phi)/cos(obj.theta));

            obj.phi = obj.phi + phi_dot/obj.samplingFrequency;
            obj.theta = obj.theta + theta_dot/obj.samplingFrequency;
            obj.psi = obj.psi + psi_dot/obj.samplingFrequency;

            A = [g(2)*cos(obj.phi)*tan(obj.theta)-g(3)*sin(obj.phi)*tan(obj.theta), g(2)*sin(obj.phi)/(cos(obj.theta)^2)+g(3)*cos(obj.phi)/(cos(obj.theta)^2),0;
                 -g(2)*sin(obj.phi)-g(3)*cos(obj.phi), 0, 0;
                 g(2)*cos(obj.phi)/cos(obj.theta)-g(3)*sin(obj.phi)/cos(obj.theta), g(2)*sin(obj.theta)*sin(obj.phi)/(cos(obj.theta)^2)+g(3)*sin(obj.theta)*cos(obj.phi)/(cos(obj.theta)^2),0];

            obj.P = obj.P+(A*obj.P+obj.P*transpose(A)+obj.Q)/obj.samplingFrequency;
            %Predict End
            %Update start-Euler
             R_x = [1,0,0;
                    0, cos(-obj.theta), -sin(-obj.theta);
                    0, sin(-obj.theta), cos(-obj.theta)];
            R_y = [cos(obj.phi), 0, sin(obj.phi);
                   0, 1, 0;
                   -sin(obj.phi), 0, cos(obj.phi)];
            m_prime = R_x*R_y*m;
            m_prime(3) = 0;
            mag = (m_prime(1)^2+m_prime(2)^2)^0.5;
            m_prime = m_prime/mag;
            h = -[-sin(obj.theta);cos(obj.theta)*sin(obj.phi);cos(obj.phi)*cos(obj.theta);-sin(obj.psi);-cos(obj.psi)];
            C = -[0, -cos(obj.theta), 0;
                 cos(obj.theta)*cos(obj.phi), -sin(obj.theta)*sin(obj.phi), 0;
                 -sin(obj.phi)*cos(obj.theta), -cos(obj.phi)*sin(obj.theta), 0;
                 0, 0, -cos(obj.psi);
                 0, 0, sin(obj.psi)];%dont forget, there is - in front of the full matrix
            K = obj.P*transpose(C)/(C*obj.P*transpose(C)+obj.R);
            temp = K*(vertcat(a,m_prime(1),m_prime(2))-h);
            obj.phi = obj.phi + temp(1);
            obj.theta = obj.theta + temp(2);
            obj.psi = obj.psi + temp(3);
            obj.P = (eye(3)-K*C)*obj.P;
            %Update end
            end

            roll = obj.phi*180.0/3.14;
            pitch = obj.theta*180.0/3.14;
            yaw = obj.psi*180.0/3.14;

            %Just keeping the rotation matrix here in case ai ever need it
            %again
%             R_to_body = [cos(obj.psi)*cos(obj.theta), cos(obj.theta)*sin(obj.psi), -sin(obj.theta);
%                          cos(obj.psi)*sin(obj.phi)*sin(obj.theta)-cos(obj.phi)*sin(obj.psi), cos(obj.phi)*cos(obj.psi)+sin(obj.phi)*sin(obj.psi)*sin(obj.theta), cos(obj.theta)*sin(obj.phi);
%                          sin(obj.phi)*sin(obj.psi)+cos(obj.phi)*cos(obj.psi)*sin(obj.theta), cos(obj.phi)*sin(obj.psi)*sin(obj.theta)-cos(obj.psi)*sin(obj.phi), cos(obj.phi)*cos(obj.theta)];
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end

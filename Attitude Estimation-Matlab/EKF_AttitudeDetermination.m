classdef EKF_AttitudeDetermination < matlab.System
    % untitled Add summary here
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
    q;
    P;
    Q;
    R;
    end

    % Pre-computed constants
    properties (Access = private)

    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.q = [1;
                     0;
                     0;
                     0];
            obj.P = eye(4)*0.1;
            obj.Q = eye(4)*0.001;
            obj.R = eye(6)*0.011;
        end

        function [roll,pitch,yaw] = stepImpl(obj,accelerometer, gyroscope, magnetometer)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            g = [gyroscope(1)-obj.gyro_y_corr;
                 gyroscope(2)-obj.gyro_x_corr;
                 gyroscope(3)-obj.gyro_z_corr];
            a = [accelerometer(1);
                 accelerometer(2);
                 accelerometer(3)];
            mag = (a(1)^2+a(2)^2+a(3)^2)^0.5;
            a = a/mag;
            m = [magnetometer(1);
                 -magnetometer(2);
                 -magnetometer(3)];
            %Predict start-Quaternion
            special_matrix = [-obj.q(2), -obj.q(3), -obj.q(4);
                              obj.q(1), -obj.q(4), obj.q(3);
                              obj.q(4), obj.q(1), -obj.q(2);
                              -obj.q(3), obj.q(2), obj.q(1)];
            q_dot = 0.5*special_matrix*g;
            obj.q = obj.q + q_dot/obj.samplingFrequency;
            mag = (obj.q(1)^2+obj.q(2)^2+obj.q(3)^2+obj.q(4)^2)^0.5;
            obj.q = obj.q/mag;
            A = 0.5*[0, -g(1), -g(2), -g(3);
                     g(1), 0, g(3), -g(2);
                     g(2), -g(3), 0, g(1);
                     g(3), g(2), -g(1), 0];
            obj.P = obj.P + (A*obj.P + obj.P*transpose(A) + obj.Q)/obj.samplingFrequency;
            %Predict end

            %Update start-Quaternion
            R_to_ref = [obj.q(1)^2+obj.q(2)^2-obj.q(3)^2-obj.q(4)^2, 2*obj.q(2)*obj.q(3)-2*obj.q(1)*obj.q(4), 2*obj.q(2)*obj.q(4)+2*obj.q(1)*obj.q(3);
                         2*obj.q(2)*obj.q(3)+2*obj.q(1)*obj.q(4), obj.q(1)^2-obj.q(2)^2+obj.q(3)^2-obj.q(4)^2, 2*obj.q(3)*obj.q(4)-2*obj.q(1)*obj.q(2);
                         2*obj.q(2)*obj.q(4)-2*obj.q(1)*obj.q(3), 2*obj.q(3)*obj.q(4)+2*obj.q(1)*obj.q(2), obj.q(1)^2-obj.q(2)^2-obj.q(3)^2+obj.q(4)^2];
              m_prime = R_to_ref*m;
              m_prime(3) = 0;
              m_prime = m_prime/((m_prime(1)^2+m_prime(2)^2)^0.5);
              m_prime = transpose(R_to_ref)*m_prime;
              h = vertcat(transpose(R_to_ref)*[0;0;1],transpose(R_to_ref)*[0;1;0]);

            C = 2*[-obj.q(3), obj.q(4), -obj.q(1), obj.q(2);
                   obj.q(2), obj.q(1), obj.q(4), obj.q(3);
                   obj.q(1), -obj.q(2), -obj.q(3), obj.q(4);
                   obj.q(4), obj.q(3), obj.q(2), obj.q(1);
                   obj.q(1), -obj.q(2), obj.q(3), -obj.q(4);
                   -obj.q(2), -obj.q(1), obj.q(4), obj.q(3)];
            K = obj.P*transpose(C)/(C*obj.P*transpose(C) + obj.R);
            obj.q = obj.q + K*(vertcat(a,m_prime)-h);
            mag = (obj.q(1)^2+obj.q(2)^2+obj.q(3)^2+obj.q(4)^2)^0.5;
            obj.q = obj.q/mag;
            obj.P = (eye(4)-K*C)*obj.P;
%             %Update end


           temp = quat2eul(transpose(obj.q));
           yaw = -temp(1)*180.0/3.14;
           pitch = -temp(2)*180.0/3.14;
           roll = temp(3)*180.0/3.14;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end

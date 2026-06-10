// day la file code thay cho file MultiRotorPhysicsBody.hpp trong source cua AirSim



// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_multirotorphysicsbody_hpp
#define msr_airlib_multirotorphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "RotorActuator.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
#include <cmath>
#include <fstream>
#include <algorithm>

namespace msr
{
    namespace airlib
    {

        class MultiRotorPhysicsBody : public PhysicsBody
        {
        public:
            MultiRotorPhysicsBody(MultiRotorParams* params, VehicleApiBase* vehicle_api,
                Kinematics* kinematics, Environment* environment)
                : params_(params), vehicle_api_(vehicle_api)
            {
                setName("MultiRotorPhysicsBody");
                vehicle_api_->setParent(this);
                initialize(kinematics, environment);
            }

            virtual void resetImplementation() override
            {
                PhysicsBody::resetImplementation();
                resetSensors();
            }

            virtual void update() override
            {
                PhysicsBody::update();
            }

            virtual void reportState(StateReporter& reporter) override
            {
                PhysicsBody::reportState(reporter);
                reportSensors(*params_, reporter);

                for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                    reporter.startHeading("", 1);
                    reporter.writeValue("Rotor", rotor_index);
                    reporter.endHeading(false, 1);
                    rotors_.at(rotor_index).reportState(reporter);
                }
            }

            //-----------------------------------------------------------------------------
		CODE BAT DAU TU DAY
		//------------------------------------------------------
            void injectAerodynamics()
            {
                Vector3r velocity_world = getKinematics().twist.linear;
                Vector3r velocity_body = VectorMath::transformToBodyFrame(
                    velocity_world, getKinematics().pose.orientation);

                const real_T v_forward_body = velocity_body.x();
                const real_T v_lateral_body = velocity_body.y();
                const real_T v_vertical_body = velocity_body.z();

                
                const real_T v_planar_body = std::sqrt(v_forward_body * v_forward_body
                    + v_lateral_body * v_lateral_body);

                const real_T WING_AREA = 0.18f;
                const real_T AIR_DENSITY = 1.225f;
                Wrench aero_wrench = Wrench::zero();

                real_T alpha = 0.0f;
                real_T CL = 0.0f;
                real_T lift = 0.0f;
                real_T drag_x = 0.0f;

                
                if (v_planar_body > 6.0f) {
                    CL = 0.1f + ((v_planar_body - 6.0f) / 8.0f) * 0.75f;
                    if (CL > 0.85f) {
                        CL = 0.85f;
                    }

                    
                    alpha = std::atan2(-v_vertical_body, std::max(v_planar_body, (real_T)0.1f));
                    alpha = std::max((real_T)-0.12f, std::min(alpha, (real_T)0.18f));
                    
                    const real_T ALPHA_GAIN = 0.60f;
                    CL += ALPHA_GAIN * alpha;
                    CL = std::max((real_T)0.0f, std::min(CL, (real_T)0.95f));

                    lift = 0.5f * AIR_DENSITY * (v_planar_body * v_planar_body) * WING_AREA * CL;
                    aero_wrench.force.z() -= lift; 

                    
                    const real_T CD0 = 0.015f;
                    const real_T K_DRAG = 0.015f;
                    const real_T CD = CD0 + K_DRAG * CL * CL;
                    const real_T drag_mag = 0.5f * AIR_DENSITY * (v_planar_body * v_planar_body) * WING_AREA * CD;

                    
                    const real_T x_sign = (v_forward_body >= 0.0f) ? 1.0f : -1.0f;
                    drag_x = -x_sign * drag_mag;
                    aero_wrench.force.x() += drag_x;
                }

                
                if (v_planar_body > 8.0f) {
                    float frac = static_cast<float>((v_planar_body - 8.0f) / (37.0f - 8.0f));
                    
                    if (frac < 0.0f) frac = 0.0f;
                    else if (frac > 1.0f) frac = 1.0f;
                    float thrust_factor = 1.0f - 0.4f * frac; 
                    lift *= thrust_factor;
                    drag_x *= thrust_factor;
                    
                    aero_wrench.force.z() = -lift;
                    aero_wrench.force.x() = drag_x;
                }

                this->setWrench(aero_wrench);


                static bool is_file_opened = false;
                static std::ofstream aero_log;

                if (!is_file_opened) {
                    std::string file_name =
                        "C:\\Users\\Acer\\Desktop\\powertrain_log_" +
                        std::to_string(Utils::getTimeSinceEpochNanos()) + ".csv";

                    aero_log.open(file_name);
                    aero_log << "timestamp_ns,v_world_mag,v_forward_body,v_lateral_body,v_vertical_body,"
                        << "v_planar_body,alpha_deg,cl,lift_z,drag_x\n";
                    is_file_opened = true;
                }

                if (aero_log.is_open()) {
                    aero_log << Utils::getTimeSinceEpochNanos() << ","
                        << velocity_world.norm() << ","
                        << v_forward_body << ","
                        << v_lateral_body << ","
                        << v_vertical_body << ","
                        << v_planar_body << ","
                        << (alpha * 180.0f / M_PIf) << ","
                        << CL << ","
                        << lift << ","
                        << drag_x << "\n";
                    aero_log.flush();
                }


            }
			//-----------------------------------------------------------------------------
		//Code ket thuc o day
		//----------------------------------------------------

            virtual void updateKinematics(const Kinematics::State& kinematics) override
            {
                PhysicsBody::updateKinematics(kinematics);
                injectAerodynamics();
                updateSensorsAndController();
            }

            virtual void updateKinematics() override
            {
                PhysicsBody::updateKinematics();
                injectAerodynamics();
                updateSensorsAndController();
            }

            void updateSensorsAndController()
            {
                updateSensors(*params_, getKinematics(), getEnvironment());
                vehicle_api_->update();

                for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                    rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));
                }
            }

            const SensorCollection& getSensors() const { return params_->getSensors(); }
            virtual uint wrenchVertexCount() const override { return params_->getParams().rotor_count; }
            virtual PhysicsBodyVertex& getWrenchVertex(uint index) override { return rotors_.at(index); }
            virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override { return rotors_.at(index); }
            virtual uint dragVertexCount() const override { return static_cast<uint>(drag_faces_.size()); }
            virtual PhysicsBodyVertex& getDragVertex(uint index) override { return drag_faces_.at(index); }
            virtual const PhysicsBodyVertex& getDragVertex(uint index) const override { return drag_faces_.at(index); }
            virtual real_T getRestitution() const override { return params_->getParams().restitution; }
            virtual real_T getFriction() const override { return params_->getParams().friction; }

            RotorActuator::Output getRotorOutput(uint rotor_index) const
            {
                return rotors_.at(rotor_index).getOutput();
            }

            virtual ~MultiRotorPhysicsBody() = default;

        private:
            void initialize(Kinematics* kinematics, Environment* environment)
            {
                PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);
                createRotors(*params_, rotors_, environment);
                createDragVertices();
                initSensors(*params_, getKinematics(), getEnvironment());
            }

            static void createRotors(const MultiRotorParams& params, std::vector<RotorActuator>& rotors, const Environment* environment)
            {
                rotors.clear();
                for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
                    const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
                    rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
                }
            }

            void reportSensors(MultiRotorParams& params, StateReporter& reporter) { params.getSensors().reportState(reporter); }
            void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
            {
                unused(state); unused(environment);
                params.getSensors().update();
            }
            void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
            {
                params.getSensors().initialize(&state, &environment);
            }
            void resetSensors() { params_->getSensors().reset(); }

            void createDragVertices()
            {
                const auto& params = params_->getParams();
                real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
                real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;
                real_T top_bottom_area = params.body_box.x() * params.body_box.y();
                real_T left_right_area = params.body_box.x() * params.body_box.z();
                real_T front_back_area = params.body_box.y() * params.body_box.z();

                Vector3r drag_factor_unit = Vector3r(
                    front_back_area + rotors_.size() * propeller_xsection,
                    left_right_area + rotors_.size() * propeller_xsection,
                    top_bottom_area + rotors_.size() * propeller_area) *
                    params.linear_drag_coefficient / 2;

                drag_faces_.clear();
                drag_faces_.emplace_back(Vector3r(0, 0, -params.body_box.z() / 2.0f), Vector3r(0, 0, -1), drag_factor_unit.z());
                drag_faces_.emplace_back(Vector3r(0, 0, params.body_box.z() / 2.0f), Vector3r(0, 0, 1), drag_factor_unit.z());
                drag_faces_.emplace_back(Vector3r(0, -params.body_box.y() / 2.0f, 0), Vector3r(0, -1, 0), drag_factor_unit.y());
                drag_faces_.emplace_back(Vector3r(0, params.body_box.y() / 2.0f, 0), Vector3r(0, 1, 0), drag_factor_unit.y());
                drag_faces_.emplace_back(Vector3r(-params.body_box.x() / 2.0f, 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
                drag_faces_.emplace_back(Vector3r(params.body_box.x() / 2.0f, 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());
            }

        private:
            MultiRotorParams* params_;
            std::vector<RotorActuator> rotors_;
            std::vector<PhysicsBodyVertex> drag_faces_;
            std::unique_ptr<Environment> environment_;
            VehicleApiBase* vehicle_api_;
        };
    }
}
#endif

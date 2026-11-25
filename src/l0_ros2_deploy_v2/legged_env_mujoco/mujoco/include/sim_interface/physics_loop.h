#pragma once

namespace mujoco {
class Simulate;
}

void PhysicsLoop(mujoco::Simulate& sim, const char* body_name);
void PhysicsThread(mujoco::Simulate* sim, const char* filename, const char* body_name);

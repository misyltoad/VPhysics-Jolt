//=================================================================================================
//
// This is the precompiled header
//
//=================================================================================================

#pragma once

// Tier0
#include "tier0/basetypes.h"
#include "tier0/dbg.h"

// Workaround mem.h #defining offsetof
// on public SDKs when on Linux.
// We don't want this behaviour.
#ifdef LINUX
#define WAS_LINUX
#undef LINUX
#endif
#include "tier0/mem.h"
#ifdef WAS_LINUX
#define LINUX
#undef WAS_LINUX
#endif

#ifndef GAME_SDK2013
#include "tier0/logging.h"
#endif

#if defined( GAME_SDK2013 )
#include "compat/compat_sdk2013.h"
#elif defined( GAME_ASW )
#include "compat/compat_asw.h"
#endif

#include "compat/branch_overrides.h"

// STD
// Ensure cmath is included everywhere
// so we get those sweet overloaded maths functions
#include <cstdlib>
#include <cmath>

// STL
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>
#include <fstream>

// Mathlib
#include "mathlib/mathlib.h"
#include "mathlib/vector.h"

// Tier1
#include "tier1/tier1.h"
#include "tier1/strtools.h"
#include "tier1/interface.h"
#ifndef GAME_L4D2_OR_NEWER
#include "tier1/KeyValues.h"
#else
#include "tier1/keyvalues.h"
#endif
#include "tier1/UtlStringMap.h"
#include "tier1/utlbuffer.h"

// Misc
#ifdef JPH_DEBUG_RENDERER
#include "engine/ivdebugoverlay.h"
#endif
#include "bspfile.h"
#include "cmodel.h"
#include "const.h"
#include "isaverestore.h"
#include "vcollide_parse.h"

// VPhysics Interface
#include "vphysics_interface.h"
#include "vphysics/collision_set.h"
#include "vphysics/constraints.h"
#include "vphysics/friction.h"
#include "vphysics/object_hash.h"
#include "vphysics/performance.h"
#include "vphysics/player_controller.h"
#include "vphysics/stats.h"
#include "vphysics/vehicles.h"
#include "vphysics/virtualmesh.h"

// Jolt
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/Factory.h>

#include <Jolt/Skeleton/Skeleton.h>

#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

// Ourselves
#include "vjolt_interface.h"
#include "vjolt_util.h"

# z1_sdk_ros ↔ unitree_arm_sdk compile notes

The `z1_sdk_ros` sources in this workspace do **not** compile as committed
against the `unitree_arm_sdk` version vendored here (`unitree_ws/z1_sdk`). The
uncommitted working-tree diff on the two nodes is an **API-compatibility shim**
for this SDK version, not a feature change. This file records the failures and
their fixes so the diff is not mistaken for removed functionality.

Affected files:
- `unitree_ros_ws/src/z1_sdk_ros/src/z1_lowcmd_node.cpp`
- `unitree_ros_ws/src/z1_sdk_ros/src/z1_panda_xy_controller.cpp`

## Root cause

The committed code was written against a **different version** of
`unitree_arm_sdk` than the one checked out here. In the vendored SDK:

- the Eigen typedefs `Vec6`, `HomoMat`, … are declared at **global scope**, not
  inside `namespace UNITREE_ARM`;
- `LowlevelCmd::kp` / `kd` are `std::vector<double>`, not fixed-size arrays;
- the low-level command/state fold the gripper into the last element of the
  7-wide arrays, so there is no separate `last_gripper_q_` member.

Building against that SDK produces three distinct compile errors.

## Error 1 — `using UNITREE_ARM::Vec6;` / `HomoMat` (both files)

`Vec6` and `HomoMat` are defined at global scope in
`z1_sdk/include/unitree_arm_sdk/math/mathTypes.h`:

```cpp
using Vec6    = typename Eigen::Matrix<double, 6, 1>;  // line 19, global scope
using HomoMat = typename Eigen::Matrix<double, 4, 4>;  // line 46, global scope
```

There is no `namespace UNITREE_ARM { … }` around them (the SDK headers open that
namespace *after* including `mathTypes.h`). So the qualified using-declarations
fail:

```cpp
using UNITREE_ARM::Vec6;     // error: 'Vec6' is not a member of 'UNITREE_ARM'
using UNITREE_ARM::HomoMat;  // error: 'HomoMat' is not a member of 'UNITREE_ARM'
```

**Fix:** remove the `using UNITREE_ARM::Vec6;` / `using UNITREE_ARM::HomoMat;`
lines. Unqualified `Vec6` / `HomoMat` still resolve to the global typedefs, so
the rest of the code is unchanged.

## Error 2 — `std::array = std::vector` (z1_lowcmd_node.cpp)

`LowlevelCmd.h:17-18` declares the gains as vectors:

```cpp
std::vector<double> kp;
std::vector<double> kd;
```

but the node stores them in `std::array<double, 7> kp_ / kd_`. Direct assignment

```cpp
kp_ = arm_._ctrlComp->lowcmd->kp;   // error: no std::array = std::vector operator
```

does not compile.

**Fix:** copy element-wise with a helper (and `#include <algorithm>` for
`std::min`):

```cpp
void copyVectorToArray(const std::vector<double>& source,
                       std::array<double, 7>& target) {
  const int count = std::min<int>(source.size(), target.size());
  for (int i = 0; i < count; ++i) target[i] = source[i];
}
...
copyVectorToArray(arm_._ctrlComp->lowcmd->kp, kp_);
copyVectorToArray(arm_._ctrlComp->lowcmd->kd, kd_);
```

## Error 3 — undeclared `last_gripper_q_` (z1_lowcmd_node.cpp)

The class has no `last_gripper_q_` member; the gripper lives in the last slot of
the 7-wide arrays (`last_q_[6]`, `last_qd_[6]`, `last_tau_[6]`). So

```cpp
last_gripper_q_ = arm_.lowstate->getGripperQ();  // error: not declared in this scope
```

**Fix:** write to the array slot instead:

```cpp
last_q_[6] = arm_.lowstate->getGripperQ();
```

## Summary of the diff

| File | Change | Reason |
|------|--------|--------|
| `z1_lowcmd_node.cpp` | drop `using UNITREE_ARM::Vec6;` | `Vec6` is global, not in the namespace |
| `z1_lowcmd_node.cpp` | add `copyVectorToArray`, `#include <algorithm>` | `kp/kd` are `std::vector`, members are `std::array` |
| `z1_lowcmd_node.cpp` | `last_gripper_q_` → `last_q_[6]` | no such member; gripper is array index 6 |
| `z1_panda_xy_controller.cpp` | drop `using UNITREE_ARM::Vec6;`/`HomoMat;` | both are global, not in the namespace |

These edits only adapt to the vendored SDK's API; behavior is unchanged. If the
workspace is ever rebuilt against an SDK version where these types are
namespaced and the gains are arrays, the shim can be reverted.

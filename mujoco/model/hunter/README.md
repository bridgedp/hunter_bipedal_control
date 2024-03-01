# Hunter Description (MJCF)

Requires MuJoCo 2.2.2 or later.

## Overview

This package contains a simplified robot description (MJCF) of the [hunter](https://www.bridgedp.com/) developed by [桥介数物](https://www.bridgedp.com/). It is derived from the [publicly available
URDF
description](https://github.com/bridgedp/hunter_bipedal_control/tree/main/legged_examples/legged_hunter/legged_hunter_description).



## URDF → MJCF derivation steps

1. Manually edited the MJCF to extract common properties into the `<default>` section.
2. Manually designed collision geometries.
3. Softened the contacts of the feet to approximate the effect of rubber and
   increased `impratio` to reduce slippage.
4. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.


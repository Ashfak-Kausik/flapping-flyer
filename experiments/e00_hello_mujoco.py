"""
e00_hello_mujoco.py  —  STAGE 0 CHECK

Smallest possible MuJoCo program: build a trivial model in memory, step it,
and confirm the install works. If this runs, the environment is good.
"""
import mujoco

XML = """
<mujoco>
  <worldbody>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

print("MuJoCo version :", mujoco.__version__)
print("start height   :", round(float(data.qpos[2]), 4), "m")
for _ in range(100):            # 100 steps of free fall under gravity
    mujoco.mj_step(model, data)
print("after 100 steps:", round(float(data.qpos[2]), 4), "m  (should have fallen)")
print("OK — environment works.")
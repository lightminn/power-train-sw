"""Run in Isaac Sim Script Editor after opening the latest robot USD.

Then press Play. Sensor samples are available as JETIN_SENSOR_RIG.collect();
JETIN_SENSOR_RIG.save('/absolute/output/path') saves RGB/depth/point clouds.
"""
import sys,builtins
from pathlib import Path
import omni.usd
import runpy

stage=omni.usd.get_context().get_stage()
candidates=[]
for layer in stage.GetUsedLayers():
    if layer.realPath:
        folder=Path(layer.realPath).parent
        candidates.extend((folder,folder.parent))
if '__file__' in globals():candidates.insert(0,Path(__file__).resolve().parents[1])
package=next((p for p in candidates if (p/'config/virtual_sensors.json').is_file()),None)
assert package is not None, 'Open a USD referencing the latest JETIN package first'
sys.path.insert(0,str(package/'tools'))
from virtual_sensors import VirtualSensorRig
if hasattr(builtins,'JETIN_SENSOR_RIG'):builtins.JETIN_SENSOR_RIG.close()
JETIN_SENSOR_RIG=VirtualSensorRig()
builtins.JETIN_SENSOR_RIG=JETIN_SENSOR_RIG
runpy.run_path(str(package/'tools/install_motion_feedback.py'))
print('JETIN virtual sensors initialized:',list(JETIN_SENSOR_RIG.cameras))

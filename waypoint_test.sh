source setup.sh
wait
(rosrun geofencer breach_detector.py ./SimulationFiles/Demo\ Simulations/pulchowk_polys1_utm_buffer.geojson & rosrun path_planner waypoint.py ./SimulationFiles/Demo\ Simulations/pulchowk_polys1_utm_buffer.geojson)
#rosrun path_planner waypoint_no_runtime.py ./SimulationFiles/Demo\ Simulations/pulchowk_polys1_utm_buffer.geojson

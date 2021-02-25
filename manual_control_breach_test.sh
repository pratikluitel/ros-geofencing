source setup.sh
wait
(rosrun geofencer breach_detector.py ./SimulationFiles/Demo\ Simulations/pulchowk_polys1_utm_buffer.geojson & rosrun geofencer control.py)

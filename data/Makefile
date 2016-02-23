
all: fuse

bag_to_csv: trial.bag
	rosrun multi_robot_fusion bag_to_csv.py trial.bag

fix_times: bag_to_csv
	rosrun multi_robot_fusion fix_pose_times.py

static_tf: ../config/static_tf.yml

../config/static_tf.yml: ../config/calibration.yml calibration/observer/observer_calibration.bag calibration/picket_1/picket_1_calibration.bag calibration/picket_2/picket_2_calibration.bag
	cd calibration/observer && rosrun multi_robot_fusion bag_to_csv.py observer_calibration.bag && \
	cd ../../calibration/picket_1 && rosrun multi_robot_fusion bag_to_csv.py picket_1_calibration.bag && \
	cd ../../calibration/picket_2 && rosrun multi_robot_fusion bag_to_csv.py picket_2_calibration.bag && \
	cd ../../ && rosrun multi_robot_fusion ot_csv_to_yml.py

construct: fix_times ../config/static_tf.yml
	rosrun multi_robot_fusion construct_poses.py

fuse: construct
	rosrun multi_robot_fusion fuse_poses.py

plot: tf-observer_exp-picket_1_exp.csv 
	rosrun multi_robot_fusion plot_data.py


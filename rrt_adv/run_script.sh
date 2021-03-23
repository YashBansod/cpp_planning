set -x
./cmake-build-release/rrt_adv -s "(30,-35,90)" -g "(0,0,5)" -e 10 -o ./inputs/obstacles.txt -m ./inputs/robot.txt -p ./outputs/path_output_1.txt -t ./outputs/search_output_1.txt -v -r 20
./cmake-build-release/post_process -i ./outputs/path_output_1.txt -o ./outputs/traj_output_1.txt -v

./cmake-build-release/rrt_adv -s "(40,-40,0)" -g "(30,40,5)" -e 5 -o ./inputs/obstacles.txt -m ./inputs/robot.txt -p ./outputs/path_output_2.txt -t ./outputs/search_output_2.txt -v -r 20
./cmake-build-release/post_process -i ./outputs/path_output_2.txt -o ./outputs/traj_output_2.txt -v

./cmake-build-release/rrt_adv -s "(30,40,270)" -g "(-50,-30,5)" -e 5 -o ./inputs/obstacles.txt -m ./inputs/robot.txt -p ./outputs/path_output_3.txt -t ./outputs/search_output_3.txt -v -r 20
./cmake-build-release/post_process -i ./outputs/path_output_3.txt -o ./outputs/traj_output_3.txt -v

./cmake-build-release/rrt_adv -s "(-50,-30,90)" -g "(30,-35,5)" -e 3 -o ./inputs/obstacles.txt -m ./inputs/robot.txt -p ./outputs/path_output_4.txt -t ./outputs/search_output_4.txt -v -r 20
./cmake-build-release/post_process -i ./outputs/path_output_4.txt -o ./outputs/traj_output_4.txt -v

./cmake-build-release/rrt_adv -s "(-30,-35,45)" -g "(-35,30,5)" -e 2 -o ./inputs/obstacles.txt -m ./inputs/robot.txt -p ./outputs/path_output_5.txt -t ./outputs/search_output_5.txt -v -r 20
./cmake-build-release/post_process -i ./outputs/path_output_5.txt -o ./outputs/traj_output_5.txt -v
set +x

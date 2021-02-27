set -x
./cmake-build-release/rrt_2d -s "(0,0)" -g "(-38,20,10)" -e 10 -o ./inputs/obstacles.txt -p ./outputs/path_output_1.txt -t ./outputs/search_output_1.txt -v
./cmake-build-release/rrt_2d -s "(27,30)" -g "(-48,20,10)" -e 5 -o ./inputs/obstacles.txt  -p ./outputs/path_output_2.txt -t ./outputs/search_output_2.txt -v
./cmake-build-release/rrt_2d -s "(45,-45)" -g "(-45,45,15)" -e 5 -o ./inputs/obstacles.txt  -p ./outputs/path_output_3.txt -t ./outputs/search_output_3.txt -v
./cmake-build-release/rrt_2d -s "(-16,10)" -g "(18,-45,5)" -e 2 -o ./inputs/obstacles.txt  -p ./outputs/path_output_4.txt -t ./outputs/search_output_4.txt -v
./cmake-build-release/rrt_2d -s "(39,5)" -g "(38,-8,3)" -e 1 -o ./inputs/obstacles.txt  -p ./outputs/path_output_5.txt -t ./outputs/search_output_5.txt -v
set +x

set -x
./cmake-build-release/rrt_2d_adv -s "(0,0)" -g "(-38,20,10)" -e 10 -o ./inputs/obstacles.txt -p ./outputs/path_output_1.txt -t ./outputs/search_output_1.txt -v -r 20
set +x


function clean_up {
	echo "Cleaning up: terminating child processes"
	pkill -P $$
	exit
}

trap clean_up SIGHUP SIGINT SIGTERM

echo "test propagating SIGTERM" $$
rostopic pub /crystalfontz/line_1 std_msgs/String "data: 'TEST 1 result                 '" &
rostopic pub /crystalfontz/line_2 std_msgs/String "data: 'TEST 1 result                 '" &
wait

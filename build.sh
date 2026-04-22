#!/bin/sh

build_module() {
    make COMPAT32=yes
    sudo make install
    make clean
    sudo kldload ntsync
}

build_tests() {
    cc -pthread -O2 -Wall -Wextra -I/usr/local/include ${PWD}/tests/ntsync_full_multithreaded_test.c -o ${PWD}/bin/ntsync_full_multithreaded_test
    cc -O2 -Wall -Wextra -I/usr/local/include ${PWD}/tests/ntsync_create_wait.c -o ${PWD}/bin/ntsync_create_wait
    cc -O2 -Wall -Wextra -I/usr/local/include ${PWD}/tests/ntsync_full_test.c -o ${PWD}/bin/ntsync_full_test
    cc -O2 -Wall -Wextra -I/usr/local/include ${PWD}/tests/ntsync_test.c -o ${PWD}/bin/ntsync_test
    cc -O2 -Wall -Wextra -I/usr/local/include ${PWD}/tests/ioctl_test.c -o ${PWD}/bin/ioctl_test
}

run_tests() {
    ${PWD}/bin/ioctl_test
    ${PWD}/bin/ntsync_create_wait
    ${PWD}/bin/ntsync_test /dev/ntsync
    ${PWD}/bin/ntsync_full_test /dev/ntsync
    ${PWD}/bin/ntsync_full_multithreaded_test /dev/ntsync
}

run_debug() {
    local child_pid="${1}"

    WINEDEBUG=+sync,+server wine wineboot 2>&1 | head -200 &> winelog.txt
    sudo ktrace -f ktrace.out ${PWD}/bin/ntsync_full_test
    sudo kdump -f ktrace.out > kdump.txt
    sudo procstat -k ${child_pid}
    procstat -t ${child_pid}
}

#build_module
#build_tests
#run_tests
#run_debug <child-pid-arg>

clear
load map1
bug = Bug1(map1);
start = [40,40];
goal = [100,120];
bug.query(start,goal,'animate');

load map2
bug = Bug1(map2);
start = [10,30];
goal = [120,110];
bug.query(start,goal,'animate');

load map3
bug = Bug1(map3);
start = [100,100];
goal = [5,30];
bug.query(start,goal,'animate');
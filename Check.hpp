#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <cstring>
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <list>
#include <iterator>
using namespace std;
namespace Check {
#define UP 0
#define RIGHT 1
#define DOWN 2
#define STRAIGHT 2
#define LEFT 3

#define DEBUG 1
//    #define schedulingDEBUG 1
//    #define arrangeOnRoadDEBUG 1
//    #define processCarStatusDEBUG 1
//    #define constructSchedulingUnitDEBUG 1
//    #define endDEBUG
    //#define CheckReadFun 1
    //#define CheckUpdateFun 1
    typedef long long ll;

    const int maxn = 1e6 + 10;
    const int waitingStatus = 1;
    const int endStatus = 2;
    unordered_map<int, int> car_map;
    unordered_map<int, int> road_map;
    unordered_map<int, int> cross_map;
    int car_index, car_hashBack[maxn];
    int road_index, road_hashBack[maxn];
    int cross_index, cross_hashBack[maxn];
    int dirHash[4][4] = {
            {-1, LEFT, STRAIGHT, RIGHT},
            {RIGHT, -1, LEFT, STRAIGHT},
            {STRAIGHT, RIGHT, -1, LEFT},
            {LEFT, STRAIGHT, RIGHT, -1}
    };
    int dirCheck[4][4][2] = {
            {{-1, -1}, {3, -1}, {-1, -1}, {1, 2}},
            {{2, 3}, {-1, -1}, {0, -1}, {-1, -1}},
            {{-1, -1}, {3, 0}, {-1, -1}, {1, -1}},
            {{2, -1}, {-1, -1}, {0, 1}, {-1, -1}}
    };

    int systemTime = 0;         // system scheduling time
    int totTimeUsed = 0;
    double checkerRunTime = 0;
    int curCarOnRoadId = 0;     // index of car should go on the road
    int totCarCounter = 0;
    int onRoadCarsCounter = 0;
    int waitingCarsCounter = 0;
    int lastWaitingCarsCounter = 0;
    int inGarageCarsCounter = 0;
    bool isDeadLock = false;
    bool isAllCarsReach = false;
    list<int> waitingList;      // the carId have to wait to go on the road
    set<int> isCarAdded;
    set<int> deadLockCrossId;
    set<int> deadLockRoadId;
    set<int> carcheck;
    set<int> carcheck2;
    set<int> waitingListSet;
    struct Car {
        /* Cars' argv about itself*/
        int id;
        int from;           // src cross id
        int to;             // dst cross id
        int speed;
        int status;
        int planTime;
        int RealStartTime;

        /* Cars' argv about roads*/
        int curRoadId;      // currently car is on which road
        int curChannelId;   // currently car is on which channel
        int curPosition;    // currently car is where on the oad
        int curPathId;      // current path index
        int nextCrossId;    // car will be which cross next;
        vector<int> path;   // ans of paths

        Car() {
            status = curPathId = 0;
            id = from = to = speed = planTime = RealStartTime = curRoadId = curChannelId = curPosition = nextCrossId = -1;
        }

        Car(const int & _id, const int & _from, const int & _to, const int & _speed, const int & _planTime) {
            id = _id;
            to = _to;
            nextCrossId = from = _from;
            speed = _speed;
            planTime = _planTime;
            status = curPathId = 0;
            RealStartTime = curRoadId = curChannelId = curPosition  = -1;
        }

        bool operator < (const Car &x) const {
            if (RealStartTime != x.RealStartTime) return RealStartTime < x.RealStartTime;
            return id < x.id;
        }

        void reset() {
            nextCrossId = from;
            status = curPathId = 0;
            curRoadId = curChannelId = curPosition  = -1;
        }

        int selfcheck(bool is_show_message) {
            if (id == -1 || from == -1 || to == -1 || speed == -1 || planTime == -1) {
                if (is_show_message)
                    cout << "[ERROR]   [Car self check]: cars_" << id << " may not initialize correctly !" << endl;
                return  -1;
            }
            else if (RealStartTime == -1) {
                if (is_show_message)
                    cout << "[WARNING] [Car self check]: cars_" << id << " may not be arranged to start !" << endl;
                return 0;
            }
            else if (curRoadId == -1 || curChannelId == -1 || curPosition == -1) {
                if (is_show_message)
                    cout << "[WARNING] [Car self check]: cars_" << id << " may not be on the road !" << endl;
                return 1;
            }
            else {
                if (is_show_message)
                    cout << "[ERROR]   [Car self check]: self check ok!" << endl;
                return 2;
            }

        }

        void output() {
            cout << car_hashBack[id] << " " << cross_hashBack[from] << " " << cross_hashBack[to]
                 << " " << speed << " " << planTime << " " << RealStartTime << endl;
        }

        int nextRoadId() { return path[curPathId + 1];}
        void outAnswer() {
            cout << car_hashBack[id] << " ";
            for(int i = 0 ; i < path.size(); ++i) {
                cout << road_hashBack[path[i]] << " ";
            }
            cout << endl;
        }
        void updateNextCrossId();
    };
    struct Unit {
        int carId;
        int schedulingOrder;

        bool operator < (const Unit & rhs) const { return schedulingOrder < rhs.schedulingOrder;}
        Unit(){}
        Unit(const int & _carId, const int & _schedulingOrder) {
            carId = _carId;
            schedulingOrder = _schedulingOrder;
        }
    };
    struct Road {
        int id;
        int length;
        int speed;
        int channelNum;
        int from;       // src cross id
        int to;         // dst cross id
        bool isDuplex;

        /* Appointment
        * RoadCars[0 ~ channelNum/2 - 1] corresponding to (from -> to) direction
        * RoadCars[channelNum/2 ~ channelNum -1] corresponding to (to -> from) direction if isDuplex is true
        * */
        vector<list<int> > RoadCars;
        vector<Unit> schedulingUnit[2];

        Road() {}

        Road(const int & _id, const int & _length, const int &  _speed, const int & _channelNum,
             const int &  _from, const int & _to, const bool & _isDuplex) {
            id = _id;
            length = _length;
            speed = _speed;
            channelNum = _channelNum;
            from = _from;
            to = _to;
            isDuplex = _isDuplex;
            if(isDuplex) channelNum = channelNum << 1;
            /* initialization for cars' vector*/
            for (int i = 0; i < channelNum; ++i) {
                RoadCars.push_back(list<int>());
            }
        }

        int getFrontCarId(const int & channel, const int & backCarId) {
            if(RoadCars[channel].size() == 1) return -1;    // the first car
            int frontCarId = -1;
            for(auto & x : RoadCars[channel]) {
                if(x == backCarId) break;
                frontCarId = x;
            }
#ifdef DBUEG
            assert(frontCarId != -1);
#endif
            return frontCarId;
        }

        bool removeCarFromRoadCars(const int & channelId, const int & carId) {
            for(list<int>::iterator it =  RoadCars[channelId].begin() ; it != RoadCars[channelId].end(); ++it) {
                if(*it  == carId)  {
                    RoadCars[channelId].erase(it);
                    return true;
                }
            }
            return false;
        }

        void output() {
            cout << road_hashBack[id] << " " << length << " " << speed << " " << channelNum << " "
                 << cross_hashBack[from] << " " << cross_hashBack[to] << " " << isDuplex << endl;
        }

        bool isSameDirection(const int & crossId);
        void constructSchedulingUnit();
    };
    struct Cross {
        int id;
        int order[4];
        int connect[4]; // 0 up / 1 right / 2 down / 3 left
        unordered_map<int, int> hashPosition;

        void output() {
            cout  << cross_hashBack[id] << " ";
            for(int i = 0; i < 4; ++i) cout << (connect[i] == -1 ? -1 : road_hashBack[connect[i]]) << " ";
            cout << endl;
        }

        void update() {
            hashPosition.clear();
            for(int i = 0; i < 4; ++i) {
                if(connect[i] != -1) {
                    hashPosition[connect[i]] = i;
                }
            }
        }
        inline int get_Direction(const int nextRoadId) {
            for(int i = 0; i < 4; ++i) {
                if(connect[i] == nextRoadId)
                    return i;
            }
        }
        inline int get_Road(const int & DIR) { return connect[DIR]; }
        inline int get_Position(const int & roadId) { return hashPosition.count(roadId) == 0 ? -1 : hashPosition[roadId]; }
    };


    vector<Car> cars, cars_tmp;
    vector<Road> roads, roads_tmp;
    vector<Cross> crosses, crosses_tmp;
    vector<vector<int> >crossOrder;
    unordered_map<int, vector<pair<int, int> > >roadOrder;

    void Car::updateNextCrossId() {
        nextCrossId = roads[path[curPathId]].from == nextCrossId ? roads[path[curPathId]].to : roads[path[curPathId]].from;
    }
    void Road::constructSchedulingUnit() {
        int offset = -1;
        schedulingUnit[0].clear();
        schedulingUnit[1].clear();
        if(isDuplex) {
            offset = 0 ;
            for(int i = 0 ; i < channelNum / 2; ++i) {
                int constBase = (i - offset) * length;
                if(!RoadCars[i].empty()) {
                    for(auto & x : RoadCars[i]) {
                        if(cars[x].status == waitingStatus) {
                            schedulingUnit[0].push_back(Unit(x, constBase + cars[x].curPosition));
                            //                        cout << "test:::::::: " << car_hashBack[x] << " " << constBase + cars[x].curPosition << endl;
                            ++ waitingCarsCounter;
                        }
                    }
                }

            }
            offset = channelNum / 2;
            for(int i = channelNum / 2 ; i  < channelNum; ++i) {
                int constBase = (i - offset) * length;
                if(!RoadCars[i].empty()) {
                    for (auto &x : RoadCars[i]) {
                        if (cars[x].status == waitingStatus) {
                            schedulingUnit[1].push_back(Unit(x, constBase + cars[x].curPosition));
                            //                        cout << "test:::::::: " << car_hashBack[x] << " " << constBase + cars[x].curPosition << endl;
                            ++ waitingCarsCounter;
                        }
                    }
                }
            }
            sort(schedulingUnit[0].begin(), schedulingUnit[0].end());
            sort(schedulingUnit[1].begin(), schedulingUnit[1].end());
        } else {
            offset = 0;
            for(int i = 0; i < channelNum; ++i) {
                int constBase = (i - offset) * length;
                if(!RoadCars[i].empty()) {
                    for (auto &x : RoadCars[i]) {
                        if (cars[x].status == waitingStatus) {
                            schedulingUnit[0].push_back(Unit(x, constBase + cars[x].curPosition));
                            //                        cout << "test:::::::: " << car_hashBack[x] << " " << constBase + cars[x].curPosition << endl;
                            ++ waitingCarsCounter;
                        }
                    }
                }
            }
            sort(schedulingUnit[0].begin(), schedulingUnit[0].end());
        }
    }

    inline bool cmp_carHash(const Car &C1, const Car &C2) { return C1.id < C2.id; }
    inline bool cmp_roadHash(const Road &R1, const Road &R2) { return R1.id < R2.id; }
    inline bool cmp_crossHash(const Cross &C1, const Cross &C2) { return C1.id < C2.id; }

    void read_car(const string & path) {
        road_map[-1] = -1;
        ifstream in(path);
        string info; char ch;
        int id, from, to, speed, planTime;
        getline(in, info);  // ignore the first line of input
        cars.clear();
        while (getline(in, info)) {
            stringstream car_stream(info);
            car_stream >> ch;
            car_stream >> id;
            car_stream >> ch;
            car_stream >> from;
            car_stream >> ch;
            car_stream >> to;
            car_stream >> ch;
            car_stream >> speed;
            car_stream >> ch;
            car_stream >> planTime;
            cars_tmp.push_back(Car(id, from, to, speed, planTime));
        }
        sort(cars_tmp.begin(), cars_tmp.end(), cmp_carHash);
        car_index = 0;
        for (auto & x : cars_tmp) {
            car_map[x.id] = car_index;
            car_hashBack[car_index] = x.id;
            x.id = car_index++;
            cars.push_back(x);
        }
        cars_tmp.clear();
    }
    void read_road(const string & path) {
        ifstream in(path);
        string info; char ch;
        int id, length, speed, channelnum, from, to ;
        bool isDuplex;
        getline(in, info);  // ignore the first line of input
        roads.clear();
        while (getline(in, info)) {
            stringstream road_stream(info);
            road_stream >> ch;
            road_stream >> id;
            road_stream >> ch;
            road_stream >> length;
            road_stream >> ch;
            road_stream >> speed;
            road_stream >> ch;
            road_stream >> channelnum;
            road_stream >> ch;
            road_stream >> from;
            road_stream >> ch;
            road_stream >> to;
            road_stream >> ch;
            road_stream >> isDuplex;
            roads_tmp.push_back(Road(id, length, speed, channelnum, from, to, isDuplex));
        }
        sort(roads_tmp.begin(), roads_tmp.end(), cmp_roadHash);
        road_index = 0;
        for (auto & x : roads_tmp) {
            road_map[x.id] = road_index;
            road_hashBack[road_index] = x.id;
            x.id = road_index++;
            roads.push_back(x);
        }
        roads_tmp.clear();
    }
    void read_cross(const string & path) {
        ifstream in(path);
        string info; char ch;
        getline(in, info);          // ignore the first line of input
        crosses.clear();
        while (getline(in, info)) {
            Cross tmp;
            stringstream cross_steam(info);
            cross_steam >> ch;
            cross_steam >> tmp.id;
            cross_steam >> ch;
            for (int i = 0; i < 4; ++i) {
                cross_steam >> tmp.connect[i];
                tmp.connect[i] = road_map[tmp.connect[i]];
                tmp.order[i] = tmp.connect[i];
                tmp.hashPosition[tmp.connect[i]] = i;
                cross_steam >> ch;  // read no use char
            }
            sort(tmp.order, tmp.order + 4);
            tmp.update();
            crosses_tmp.push_back(tmp);
        }
        sort(crosses_tmp.begin(), crosses_tmp.end(), cmp_crossHash);
        cross_index = 0;
        for (auto & x : crosses_tmp) {
            cross_map[x.id] = cross_index;
            cross_hashBack[cross_index] = x.id;
            x.id = cross_index++;
            crosses.push_back(x);
        }
        crosses_tmp.clear();
    }
    void read_answer(const string & path) {
        ifstream in(path);
        string info; char ch;
        int tmp_roadId, tmp_carId, tmp_startTime, carID;
        while(getline(in, info)) {
            stringstream answer_stream(info);
            answer_stream >> ch;            // scan '('
            answer_stream >> tmp_carId;
            answer_stream >> ch;            // scan ','
            answer_stream >> tmp_startTime;
            carID = car_map[tmp_carId];
            cars[carID].RealStartTime = tmp_startTime;
            cars[carID].path.clear();
            while(true) {
                answer_stream >> ch;        // scan ',' or ')'
                if(ch == ')') break;        // break when scan ')'
                answer_stream >> tmp_roadId;
                cars[carID].path.push_back(road_map[tmp_roadId]);
            }
        }
    }
    void update_RoadCar_makeMap() {
        // update Road [from, to] to hash &  update Car [from, to] to hash
        for(auto & x:cars) {
            x.from = cross_map[x.from];
            x.to = cross_map[x.to];
            x.nextCrossId = cross_map[x.nextCrossId];
        }
        for(auto & x:roads) {
            x.from = cross_map[x.from];
            x.to = cross_map[x.to];
        }
        // [Done] make matrix to describe adjacent relationship of map
        //    for (int i = 0; i < crosses.size(); ++i) {
        //        for(int j = 0; j < 4; ++j) {
        //            if (crosses[i].connect[j] != -1) {
        //                matrix[crosses[i].id][j] = roads[crosses[i].connect[j]].to == crosses[i].id ?
        //                                           roads[crosses[i].connect[j]].from : roads[crosses[i].connect[j]].to;
        //#ifdef CheckUpdateFun
        //                if(matrix[crosses[i].id][j] == roads[crosses[i].connect[j]].to) {
        //                    cout << "to is the same" << endl;
        //                } else if(matrix[crosses[i].id][j] == roads[crosses[i].connect[j]].from) {
        //                    cout << "from is the same" << endl;
        //                } else {
        //                    cout << "ERROR" << endl;
        //                    exit(233);
        //                }
        //#endif
        //            } else matrix[crosses[i].id][j] = -1;
        //        }
        //    }
    }

    void updateCrossOrder() {
        vector<int> & cur = crossOrder[systemTime - 1];
        for(int i = 0; i < crosses.size(); ++i) {
            Cross & curCross = crosses[i];
            int tot = 0;
            for(int j = 0; j < 4; ++j) {
                if(curCross.connect[j] == 0) continue;
                else {
                    Road & curRoad = roads[curCross.connect[j]];
                    if(curRoad.to == curCross.id) tot += curRoad.schedulingUnit[0].size();
                    else tot += curRoad.schedulingUnit[1].size();
                }
            }
            cur.push_back(tot);
        }
    }
    void updateRoadOrder(const int & carId, const int & roadId, bool isin) {
        vector<pair<int, int> > & curVec = roadOrder[carId];
        if(isin) {
            assert(int(curVec.size()) % 2 == 0);
            curVec.push_back(make_pair(roadId, systemTime));
        } else {
            assert(int(curVec.size()) % 2 == 1);
            curVec.push_back(make_pair(roadId, systemTime));
        }
    }

    bool arrangeOnRoad(Car & curCar, bool isInWaitingList) {
        Road & targetRoad = roads[curCar.path[curCar.curPathId]];
        int tmp_channelId = -1, st = -1, ed = -1;
        // TODO: judge whether the road is full
        // TODO: judge whether the channel is full to arrange a reasonable channel
        // decide the order of visiting targetRoad.RoadCars[]

        if(!(targetRoad.from == curCar.from || targetRoad.to == curCar.from)) {
            cout << car_hashBack[curCar.id] << " " << road_hashBack[targetRoad.id] << endl;
            assert(curCar.curPathId == 0);
            assert(targetRoad.from == curCar.from || targetRoad.to == curCar.from);
        }
        if(curCar.from == targetRoad.from) {
            if(targetRoad.isDuplex) {
                st = 0;
                ed = targetRoad.channelNum / 2;
            } else {
                st = 0;
                ed = targetRoad.channelNum;
            }
        } else {
            st = targetRoad.channelNum / 2, ed = targetRoad.channelNum ;
        }
        // decide channelId of cars
        for(int i = st; i < ed; ++i) {
            if(!targetRoad.RoadCars[i].empty()) {
                int tmp_carId = targetRoad.RoadCars[i].back();
                if(cars[tmp_carId].curPosition == targetRoad.length) {
                    // current channel is full
                } else {
                    // current channel is not full
                    tmp_channelId = i;
                    break;
                }
            } else {
                // current channel is empty
                tmp_channelId = i;
                break;
            }
        }
        if(tmp_channelId == -1) {
            // have to wait
            if(waitingListSet.count(curCar.id) == 0) {
                waitingListSet.insert(curCar.id);
                waitingList.push_back(curCar.id);
            }
//            cout << car_hashBack[curCar.id] << "try to go on road, but faild" << endl;
            return false;
        } else {
            // TODO: decide max distance the car could drive and move
            curCar.curChannelId = tmp_channelId;
            curCar.status = endStatus;
            curCar.curRoadId = targetRoad.id;
            curCar.updateNextCrossId();
            updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curCar.curRoadId], true);
            if(targetRoad.RoadCars[tmp_channelId].empty()) {
                int maxDis = min(curCar.speed, targetRoad.speed);
                curCar.curPosition = targetRoad.length - maxDis + 1;
                targetRoad.RoadCars[tmp_channelId].push_back(curCar.id);
            } else {
                Car & frontCar = cars[targetRoad.RoadCars[tmp_channelId].back()];
                //            cout << "hahaha  FrontCarID:" << car_hashBack[frontCar.id] <<  " FrontCarPos:" << frontCar.curPosition << endl;
                int maxDis = min(min(curCar.speed, targetRoad.speed), targetRoad.length - frontCar.curPosition);
                curCar.curPosition = targetRoad.length - maxDis + 1;
                targetRoad.RoadCars[tmp_channelId].push_back(curCar.id);
            }
#ifdef arrangeOnRoadDEBUG
            cout << "[arrangeOnRoad] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[targetRoad.id]
        << ", channel:" << tmp_channelId << ". Position:" << curCar.curPosition << ". NextCrossID: " << cross_hashBack[curCar.nextCrossId] << endl;
#endif
            ++ onRoadCarsCounter;
            if(carcheck2.count(curCar.id) != 0) {
                assert(carcheck2.count(curCar.id) == 0);
                cout << car_hashBack[curCar.id] << endl;
            } else {
                carcheck2.insert(curCar.id);
            }
            return true;
        }
    }

    int processCarStatus(Car & curCar, Road & curRoad, Car & frontCar, bool isFirst) {
        if(isFirst) {
            // if the first car: whether could reach cross
            // if the car is at the front of road or speed >= left road, then waiting
            if(1 == curCar.curPosition || curCar.curPosition - 1 < min(curCar.speed, curRoad.speed)) {
                curCar.status = waitingStatus;
#ifdef processCarStatusDEBUG
                cout << "[processCarStatus] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[curRoad.id]
                << " to WAITING STATUS. Cause curCar will reach cross!" <<  " NextCross: " << cross_hashBack[curCar.nextCrossId]
                <<  " curPosition:"  << curCar.curPosition << "curChannelID:" << curCar.curChannelId << endl;
#endif
                return waitingStatus;
            } else {
                int maxMoveDistance = min(curRoad.speed, curCar.speed);
                curCar.curPosition -= maxMoveDistance;
                curCar.status = endStatus;
#ifdef processCarStatusDEBUG
                cout << "[processCarStatus] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[curRoad.id]
                 << " to END STATUS. " << "Position:" << curCar.curPosition << " NextCross:" << cross_hashBack[curCar.nextCrossId] << endl;
#endif
                return endStatus;
            }
        } else {
            // if not the first:
            // case1: curCar can catch up with frontCar
            // curCar is the same status with frontCar
            // case2: curCar can not catch up with frontCar
            // curCar is in endStatus
            int maxSpeed = min(curCar.speed, curRoad.speed);
            int frontDistance = curCar.curPosition - frontCar.curPosition - 1;
#ifdef processCarStatusDEBUG
            cout << " >> " << "frontCarID:" << car_hashBack[frontCar.id] << " frontCarPos:" << frontCar.curPosition << " frontCarDistance:" << frontDistance << " mxSpeed:" << maxSpeed << endl;
#endif
            if(maxSpeed >=  frontDistance) {
                if(frontCar.status == waitingStatus) {
#ifdef processCarStatusDEBUG
                    cout << "[processCarStatus] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[curRoad.id]
                     << " to WAITING STATUS. Cause front car " << car_hashBack[frontCar.id] << " is waiting!" << " NextCross:" << cross_hashBack[curCar.nextCrossId]
                     <<  " curPosition:"  << curCar.curPosition << "curChannelID:" << curCar.curChannelId << endl;
#endif
                    curCar.status = waitingStatus;
                    return waitingStatus;
                } else {
                    curCar.status = endStatus;
                    curCar.curPosition = frontCar.curPosition + 1;
#ifdef processCarStatusDEBUG
                    cout << "[processCarStatus] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[curRoad.id]
                     << " to END STATUS. " << "Position:" << curCar.curPosition << " NextCross:" << cross_hashBack[curCar.nextCrossId] << endl;
#endif
                    return endStatus;
                }
            } else {
                int maxMoveDistance = min(min(curRoad.speed, curCar.speed), curCar.curPosition - frontCar.curPosition - 1);
                curCar.curPosition -= maxMoveDistance;
                curCar.status = endStatus;
#ifdef processCarStatusDEBUG
                cout << "[processCarStatus] carID:" << car_hashBack[curCar.id] << ", on roadID:" << road_hashBack[curRoad.id]
                 << " to END STATUS. " << "Position:" << curCar.curPosition  << " NextCross:" << cross_hashBack[curCar.nextCrossId] << endl;
#endif
                return endStatus;
            }
        }
    }
    void decideCarsStatus(Road & curRoad) {
        list<int>::iterator it, frontIt;
        if(curRoad.isDuplex) {
            for(int i = 0; i < curRoad.channelNum / 2; ++i) {
                if(curRoad.RoadCars[i].empty()) continue;
#ifdef  processCarStatusDEBUG
                cout << "[processCarStatus] channelId:" << i << endl;
#endif
                bool isFirst = true;
                frontIt = curRoad.RoadCars[i].begin();
                for(it = curRoad.RoadCars[i].begin(); it != curRoad.RoadCars[i].end(); ++it) {
                    if(isFirst) {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                        isFirst = false;
                    } else {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                    }
                    frontIt = it;
                }
            }
            for(int i = curRoad.channelNum / 2; i < curRoad.channelNum; ++i) {
                if(curRoad.RoadCars[i].empty()) continue;
#ifdef  processCarStatusDEBUG
                cout << "[processCarStatus] channelId:" << i << endl;
#endif
                bool isFirst = true;
                frontIt = curRoad.RoadCars[i].begin();
                for(it = curRoad.RoadCars[i].begin(); it != curRoad.RoadCars[i].end(); ++it) {
                    if(isFirst) {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                        isFirst = false;
                    } else {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                    }
                    frontIt = it;
                }
            }
        } else {
            for(int i = 0; i < curRoad.channelNum; ++i) {
                if(curRoad.RoadCars[i].empty()) continue;
#ifdef  processCarStatusDEBUG
                cout << "[processCarStatus] channelId:" << i << endl;
#endif
                bool isFirst = true;
                frontIt = curRoad.RoadCars[i].begin();
                for(it = curRoad.RoadCars[i].begin(); it != curRoad.RoadCars[i].end(); ++it) {
                    if(isFirst) {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                        isFirst = false;
                    } else {
                        processCarStatus(cars[*it], curRoad, cars[*frontIt], isFirst);
                    }
                    frontIt = it;
                }
            }
        }
    }
    inline bool isCrossSameDirection(const Cross & curCross, const Road & curRoad) { return curCross.id == curRoad.to; }
    bool updateCarsStatusWhenCrossing(Car & curCar, Road & curRoad, Road & nextRoad, const int & curCrossId,
                                      const int & nextChannelId, const int & curCarNextPosition) {
        // 3. if can, calculate NEXT CHANNEL curCar will drive into, and MAX DISTANCE will move
        //    remove curCar from schedulingUnit and RoadCars, insert curCar to next RoadCars[nextChannel]
        // 4. then update curCar's status (status, curRoadId, curChannelId, curPosition, curPathId, nextCrossId)
        curRoad.removeCarFromRoadCars(curCar.curChannelId, curCar.id);
        curCar.curRoadId = nextRoad.id;
        curCar.curChannelId = nextChannelId;
        curCar.curPosition = curCarNextPosition;
        curCar.curPathId += 1;
        curCar.updateNextCrossId();
        curCar.status = endStatus;
        nextRoad.RoadCars[nextChannelId].push_back(curCar.id);
        return true;
    }
    int conflictJudgeWhenCrossing(Cross & curCross, const int & dirToCheck, const int  & curCarDir, const int & cd) {
        /*
         * @ function: check whether curCarDir conflicts with other carDir
         * @ parameters:
         * @ return:
         *          0 not conflict, it's ok to go
         *          1 conflict, must wait
         * */
        if(curCross.connect[dirToCheck] == -1) return 0;
        else {
            const Road & targetRoad = roads[curCross.connect[dirToCheck]];
            int index = isCrossSameDirection(curCross, targetRoad) ? 0 : 1;

            if(targetRoad.schedulingUnit[index].empty()) {
                return 0;
            } else {
                const Car & targetRoadFirstCar = cars[targetRoad.schedulingUnit[index][0].carId];
                if(targetRoadFirstCar.curPathId + 1 == targetRoadFirstCar.path.size()) return 1;
                const Road & nextRoad = roads[targetRoadFirstCar.path[targetRoadFirstCar.curPathId +1]];
                int curDir = curCross.get_Direction(targetRoadFirstCar.curRoadId);
                int nextDir = curCross.get_Direction(nextRoad.id);
                int dir = dirHash[curDir][nextDir];
#ifdef schedulingDEBUG
                cout << "frontCarID: " << car_hashBack[targetRoadFirstCar.id] << " ";
#endif
                if(curCarDir == LEFT) {
                    return dir == STRAIGHT ? 1 : 0;
                } else if(curCarDir == RIGHT) {
                    if(cd == 0 && dir == STRAIGHT) return 1;
                    else if(cd == 1 && dir == LEFT) return 1;
                    else return 0;
                }
                assert(curCarDir == STRAIGHT);
            }
        }
    }
    int decideWhichChannel(const Car & curCar, Road & curRoad, const Road & nextRoad, const Cross & curCross,  int & curCurPosition) {
        // return value:
        //          -1 have to wait
        //          -2 move to curRoad Position 1
        //          -3 move to curRoad other position
        int frontCarId = curRoad.getFrontCarId(curCar.curChannelId, curCar.id);
        int nextRoadMaxSpeed = min(curCar.speed, nextRoad.speed);
        int curRoadLeft = curCar.curPosition - 1;
        Car & frontCar = cars[frontCarId];
        if(frontCarId == -1) {
            // the first car
            if(nextRoadMaxSpeed - curRoadLeft <= 0) {
                // stop at the most front position of curRoad
                curCurPosition = 1;
                return -2;
            } else {
                // test nextRoad
                // TODO: judge illegal path : one direction but give wrong road
                int st = -1, ed = -1;
                if(nextRoad.isDuplex) {
                    if(curCross.id == nextRoad.from) {
                        st = 0, ed = nextRoad.channelNum / 2;
                    } else {
                        st = nextRoad.channelNum / 2, ed = nextRoad.channelNum;
                    }
                } else {
                    st = 0, ed = nextRoad.channelNum;
                }
                for(int i = st; i < ed; ++i) {
                    if(nextRoad.RoadCars[i].empty()) {
                        // ith channel of nextRoad is empty, so move to the empty channel
                        int maxDis = nextRoadMaxSpeed - curRoadLeft;
                        curCurPosition = nextRoad.length - maxDis + 1;
                        return i;
                    } else {
                        // else get last car of nextRoad i-th channel
                        Car & lastCar = cars[nextRoad.RoadCars[i].back()];
#ifdef schedulingDEBUG
                        cout << " frontCarID:" << car_hashBack[lastCar.id] << " status: " << (lastCar.status == waitingStatus? "WAITING":"END");
#endif
                        int nextRoadCarLastDis = nextRoad.length - lastCar.curPosition ;
                        int lastCarToNextRoadEnd = nextRoadCarLastDis + 1;

                        if(lastCar.status == waitingStatus) {
                            // lastCar is in waiting status
                            if(nextRoadMaxSpeed - curRoadLeft >= lastCarToNextRoadEnd) return -1; // if curCar can catch up lastCar, the waiting
                            else {
                                // else go to curCurPosition
                                curCurPosition = nextRoad.length - (nextRoadMaxSpeed - curRoadLeft) + 1;
                                return i;
                            }
                        } else if(lastCar.status == endStatus) {
                            // lastCar is in end status
                            if(lastCar.curPosition == nextRoad.length) continue; // full, then test next channel
                            else {
                                // move to correspoding position
                                int actualCurCarMoveDis = min(nextRoadMaxSpeed - curRoadLeft, nextRoadCarLastDis);
                                curCurPosition = nextRoad.length - actualCurCarMoveDis + 1;
                                return i;
                            }
                        }
                    }
                }
                // if program goes here, means nextRoad is full
                curCurPosition = 1;
                return -2;
            }
        } else {
            // have front car: move to the back of front car or as far as possible
            if(frontCar.status == endStatus) {
                int disBetweenFrontCar = curCar.curPosition - frontCar.curPosition - 1;
                int curRoadMaxSpeed = min(curRoad.speed, curCar.speed);
                int actualMoveDis = min(disBetweenFrontCar, curRoadMaxSpeed);
                curCurPosition = curCar.curPosition - actualMoveDis;
                return -3;
            } else {
                cout << "WRONG PLACE 4" << " frontCarID" << car_hashBack[frontCarId] << endl;
                exit(233);
            }
        }
    }

    void makeScheduling(Cross & curCross) {
        for(int i = 0; i < 4; ++i) {
            bool thisDirIsConflict = false;
            if(curCross.order[i] != -1) {
                Road & curRoad = roads[curCross.order[i]];
                int index = 0;
                if(curRoad.isDuplex) {
                    if(isCrossSameDirection(curCross, curRoad)) {
                        index = 0;
                    } else {
                        index = 1;
                    }
                } else if(curRoad.to == curCross.id) {
                    index = 0;
                } else {
                    continue;
                }
                if(!curRoad.schedulingUnit[index].empty()){
                    for(vector<Unit>::iterator it = curRoad.schedulingUnit[index].begin(); it != curRoad.schedulingUnit[index].end(); ) {
                        // TODO: first detect the car whether reach the dst
                        // TODO: second decide the car whether could go straight or turn
                        Car & curCar = cars[it->carId];
                        //                    cout << curCar.curPathId + 1 << " " << curCar.path.size() << endl;
                        if(curCar.curPathId + 1 == curCar.path.size()) {
                            // maybe reach dst
                            int frontCarId = curRoad.getFrontCarId(curCar.curChannelId, curCar.id);
                            if(frontCarId == -1) {
                                // the first car, so it will reach dst;
                                updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curCar.curRoadId], false);
                                curRoad.removeCarFromRoadCars(curCar.curChannelId, it->carId); // remove the car from raodCars
                                curCar.reset();
#ifdef endDEBUG
                                cout << "[TIPS] carId:" << car_hashBack[it->carId] << " reach end! Cost: " << systemTime - curCar.planTime << endl;
#endif
                                if(carcheck.count(curCar.id) != 0) {
                                    assert(carcheck.count(curCar.id) == 0);
                                    cout << car_hashBack[curCar.id] << endl;
                                } else {
                                    carcheck.insert(curCar.id);
                                }
                                totTimeUsed += systemTime - curCar.planTime;
                                it = curRoad.schedulingUnit[index].erase(it);                                // remove the car from schedulingUnit
                                onRoadCarsCounter -= 1;                                                 // minus onRoadCarsCounter
                                waitingCarsCounter -= 1;                                                // minus waitingCarsCounter
                                inGarageCarsCounter += 1;                                               // add inGarageCarsCounter

                            } else {
                                Car & frontCar = cars[frontCarId];
                                if(frontCar.status == endStatus) {
#ifdef schedulingDEBUG
                                    cout << "[SC] Ready move carID " << car_hashBack[curCar.id] << ", on RoadID" << road_hashBack[curRoad.id] <<". NextCrossID: " << cross_hashBack[curCar.nextCrossId] << ". curChannelID:" << curCar.curChannelId << endl;
                                cout << " >> Ready to move STRAIGHT.";
#endif
                                    int maxDis = min(min(curRoad.speed, curCar.speed), curCar.curPosition - frontCar.curPosition - 1);
                                    curCar.curPosition -= maxDis;
                                    curCar.status = endStatus;
                                    it = curRoad.schedulingUnit[index].erase(it);
                                    -- waitingCarsCounter;
#ifdef schedulingDEBUG
                                    cout << "Move on! Next Position:" << curCar.curPosition << endl;
#endif
                                } else if(frontCar.status == waitingStatus){
                                    cout << " WRONG PLACE 1 " << " curCarID:" << car_hashBack[curCar.id] <<  "frontCarID:" << car_hashBack[frontCarId] << endl;
                                    exit(233);
                                } else{
                                    cout << " WRONG PLACE 2" << endl;
                                    exit(233);
                                }
                                // TODO: modify the cases, refering figure  1
                                // not the first car
                                // if the front car is in waitingStatus, then curCar keeps waitingStatus
                                // otherwise, the front car should have moved from schedulingUnit !
                            }
                        } else {
                            // can not reach dst, that is curCar will go straight or turn
                            // 1. get moving(turing) direction
                            // 2. judge whether can move or turn
                            //    1) if go straight, then go
                            //    2) if turn left, check corresponding straight first-priority-car's direction
                            //    3) if turn right, check corresponding straight first-priority-car's and turn-left first-priority-car's direction
                            // 3. if can, calculate NEXT CHANNEL curCar will drive into, and MAX DISTANCE will move
                            //    remove curCar from schedulingUnit and RoadCars, insert curCar to next RoadCars[nextChannel]
                            // 4. then update curCar's status (status, curRoadId, curChannelId, curPosition, curPathId, nextCrossId)
                            // TODO: judge illegal turing case!!!
                            Road & nextRoad = roads[curCar.nextRoadId()];

                            int curDir = curCross.get_Direction(curRoad.id);
                            int nextDir = curCross.get_Direction(nextRoad.id);
                            int dir = dirHash[curDir][nextDir];
#ifdef schedulingDEBUG
                            cout << "[SC] Ready move carID " << car_hashBack[curCar.id] << ", from RoadID" << road_hashBack[curRoad.id] << " to RoadID"
                        << road_hashBack[nextRoad.id] <<". NextCrossID: " << cross_hashBack[curCar.nextCrossId] << ". curChannelID:" << curCar.curChannelId << endl;
                        cout << "dir:" << curDir << " to " << nextDir << " | ";
#endif
                            if(dir < 0 || dir > 3) {
                                cout << "dir = " << dir << "  !WTF!" << endl;
                                exit(233);
                            }
                            if(dir == STRAIGHT) {
#ifdef schedulingDEBUG
                                cout << " >> Ready to move STRAIGHT.";
#endif
                                int curCarNextPosition = -1;
                                int nextChannel = decideWhichChannel(curCar, curRoad, nextRoad, curCross, curCarNextPosition);
                                // if nextChannel == -1, then set to waiting status
                                // if nextChannel == -2, move to curRoad position 1, set to end status
                                // if nextChannel == -3, move to curRoad other position, set to end status
                                // otherwise, move to next road correspoding position, set to end status and update message
                                if(nextChannel == -1) {
#ifdef schedulingDEBUG
                                    cout << "Front car is waiting!" << " channelID:"  << curCar.curChannelId << endl;
#endif
                                    thisDirIsConflict = true;
                                    break;
                                } else if(nextChannel == -2 || nextChannel == -3) {
                                    curCar.curPosition = curCarNextPosition;
                                    curCar.status = endStatus;
                                    it = curRoad.schedulingUnit[index].erase(it);
                                    -- waitingCarsCounter;
#ifdef schedulingDEBUG
                                    cout << "Move on! Next Position:" << curCar.curPosition << endl;
#endif
                                } else {
                                    updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curRoad.id], false);
                                    updateRoadOrder(car_hashBack[curCar.id], road_hashBack[nextRoad.id], true);
                                    updateCarsStatusWhenCrossing(curCar, curRoad, nextRoad, curCross.id, nextChannel, curCarNextPosition);
                                    it = curRoad.schedulingUnit[index].erase(it);
                                    -- waitingCarsCounter;
#ifdef schedulingDEBUG
                                    cout << "Move on! Next Position:" << curCarNextPosition << endl;
#endif
                                }
                            } else if(dir == LEFT || dir == RIGHT) {
#ifdef schedulingDEBUG
                                cout << " >> Ready to move" << (dir == LEFT ? "LEFT." : "RIGHT.");
#endif
                                bool isConflict = false;
                                int dirToCheck;
                                for(int cd = 0; cd < 2; ++cd) {
                                    dirToCheck = dirCheck[curDir][nextDir][cd];
                                    if(dirToCheck == -1) break;
                                    if(conflictJudgeWhenCrossing(curCross, dirToCheck, dir, cd)) {
                                        isConflict = true;
#ifdef schedulingDEBUG
                                        cout << "Conflict! waiting!" << " channelID:"  << curCar.curChannelId << endl;
#endif
                                        break;
                                    }
                                }
                                if(!isConflict) {
                                    int curCarNextPosition = -1;
                                    int nextChannel = decideWhichChannel(curCar, curRoad, nextRoad, curCross, curCarNextPosition);
                                    if(nextChannel == -1) {
#ifdef schedulingDEBUG
                                        cout << "Front car is waiting!" << " channelID:"  << curCar.curChannelId << endl;
#endif
                                        thisDirIsConflict = true;
                                        break;
                                    } else if(nextChannel == -2 || nextChannel == -3) {
                                        curCar.curPosition = curCarNextPosition;
                                        curCar.status = endStatus;
                                        it = curRoad.schedulingUnit[index].erase(it);
                                        -- waitingCarsCounter;
#ifdef schedulingDEBUG
                                        cout << "Move on! Next Position:" << curCar.curPosition << endl;
#endif
                                    } else {
                                        updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curRoad.id], false);
                                        updateRoadOrder(car_hashBack[curCar.id], road_hashBack[nextRoad.id], true);
                                        updateCarsStatusWhenCrossing(curCar, curRoad, nextRoad, curCross.id, nextChannel, curCarNextPosition);
                                        it = curRoad.schedulingUnit[index].erase(it);
                                        -- waitingCarsCounter;
#ifdef schedulingDEBUG
                                        cout << "Move on! Next Position:" << curCarNextPosition << endl;
#endif
                                    }
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                }
                if(thisDirIsConflict)
                    continue;
            }
        }
    }

    void deadLockMessages() {
        deadLockCrossId.clear();
        deadLockRoadId.clear();
        for(int i = 0; i < crosses.size(); ++i) {
           // cout << "Ok " << cross_hashBack[i] << endl;
            for(int j = 0; j < 4; ++j) {
                if(crosses[i].order[j] != -1) {
                    Road & curRoad = roads[crosses[i].order[j]];
                    int index = isCrossSameDirection(crosses[i], curRoad) ? 0 : 1;
                    if(!curRoad.schedulingUnit[index].empty()) {
                        deadLockRoadId.insert(road_hashBack[curRoad.id]);
                        deadLockCrossId.insert(cross_hashBack[crosses[i].id]);
                    }
                }
            }
        }
    }

    void checkerReset() {

        for(auto & x : crossOrder) x.clear();
        for(auto & x : roadOrder) x.second.clear();
        crossOrder.clear();
        roadOrder.clear();
        cars_tmp.clear();
        carcheck.clear();
        carcheck2.clear();
        waitingListSet.clear();
        waitingList.clear();
        totCarCounter = isCarAdded.size();
        for(auto & x : isCarAdded) {
            cars[x].reset();
//            cars[x].nextCrossId = cars[x].from;
//            cars[x].status = cars[x].curPathId = 0;
//            cars[x].curRoadId = cars[x].curChannelId = cars[x].curPosition = -1;
            cars_tmp.push_back(cars[x]);
            roadOrder[car_hashBack[x]] = vector<pair<int, int> >();
        }
        for(auto & x : roads) {
            for(auto & y : x.RoadCars) {
                y.clear();
            }
            x.schedulingUnit[0].clear();
            x.schedulingUnit[1].clear();
        }
        sort(cars_tmp.begin(), cars_tmp.end());

        inGarageCarsCounter = 0;
        curCarOnRoadId = onRoadCarsCounter = lastWaitingCarsCounter = waitingCarsCounter  = 0;
        systemTime = 1;
        isAllCarsReach = isDeadLock = false;
    }
    void checkerInit() {
        road_map[-1] = -1;
        read_car("car.txt");
        read_road("road.txt");
        read_cross("cross.txt");
        update_RoadCar_makeMap();
    }

    void checkerRun() {
        checkerReset();
        int time1, time2;
        while (true) {
            time1 = clock();
            crossOrder.push_back(vector<int>());
            waitingCarsCounter = 0;
            // [First Round]: confirm every cars' status (waiting or end)
            for (int i = 0; i < roads.size(); ++i) {
                bool isRoadEmpty = true;
                for (auto &x : roads[i].RoadCars) {
                    if (x.empty()) continue;
                    isRoadEmpty = false;
                }
                if (isRoadEmpty) continue;
                else decideCarsStatus(roads[i]);
                roads[i].constructSchedulingUnit();
            }


            updateCrossOrder();
            // [Second Round]: move every car to its position
            lastWaitingCarsCounter  = -1;
            while (true) {
//                cout << "debug:::::::: " << inGarageCarsCounter << " " << totCarCounter << " " << onRoadCarsCounter << " " << waitingCarsCounter << endl;
                if(waitingCarsCounter == 0) {
//                    cout << "out1" << endl;
                    break;
                }

                if (onRoadCarsCounter && waitingCarsCounter == lastWaitingCarsCounter) {
//                    cout << "out2" << endl;
                    isDeadLock = true;
                    break;
                } else if (inGarageCarsCounter == totCarCounter) {
//                    cout << "out3" << endl;
                    isAllCarsReach = true;
                    break;
                }
                lastWaitingCarsCounter = waitingCarsCounter;
                for(int i = 0; i < crosses.size(); ++i)
                    makeScheduling(crosses[i]);
            }
            if(!isDeadLock) {
                for(int i  = 0; i < roads.size(); ++i) {
                    if(!roads[i].schedulingUnit[0].empty()) {
                        cout << "debug 1111111: " << endl;
                        for(auto x : roads[i].schedulingUnit[0]) {
                            cout << car_hashBack[x.carId] << " ";
                        }
                        cout << endl;
                    }
                    if(!roads[i].schedulingUnit[1].empty()) {
                        cout << "debug 2222222: " << endl;
                        for(auto x : roads[i].schedulingUnit[1]) {
                            cout << car_hashBack[x.carId] << " ";
                        }
                        cout << endl;
                    }
                    assert(roads[i].schedulingUnit[0].empty() &&roads[i].schedulingUnit[1].empty());
                }
            }

            if (isDeadLock)  {
                cout << "Dead lock!" << endl;
                deadLockMessages();
                break;
            }

            if(inGarageCarsCounter == totCarCounter)  {
                isAllCarsReach = true;
                break;
            }

            for (list<int>::iterator it = waitingList.begin(); it != waitingList.end(); ) {
                if (arrangeOnRoad(cars[*it], true)) it = waitingList.erase(it);
                else ++it;
            }
            while (cars_tmp[curCarOnRoadId].RealStartTime == systemTime) {
                assert(cars_tmp[curCarOnRoadId].RealStartTime > 1);
                int tmp_carId = cars_tmp[curCarOnRoadId].id;
                if(cars[tmp_carId].curPathId != 0)  {
                    cout << car_hashBack[tmp_carId] << endl;
                    assert(cars[tmp_carId].curPathId == 0);
                }
                arrangeOnRoad(cars[tmp_carId], false);
                ++curCarOnRoadId;
            }

            time2 = clock();
            systemTime++;
            //cout << "Processing time slice " << systemTime++ << ", cost " << (double)(time2 - time1) << " ms" << endl;
            checkerRunTime += (double)(time2 - time1);
        }
        if (isAllCarsReach) {
            assert(waitingList.empty());
            assert(carcheck.size() == totCarCounter);
           // cout << "All Car (" << inGarageCarsCounter <<") is in garage: time " << systemTime << ", totDrivingTime:"<< totTimeUsed << endl;
            //cout << "checkerRunTime: " << checkerRunTime << "ms";
        }
    }

    void modifyCar(int carId, const int & realStarTime, const vector<int> & path) {
        carId = car_map[carId];
        isCarAdded.insert(carId);
        Car & curCar = cars[carId];
        curCar.RealStartTime = realStarTime;
        curCar.path.clear();
        for(int i = 0; i < path.size(); ++i)
            curCar.path.push_back(road_map[path[i]]);
    }
}

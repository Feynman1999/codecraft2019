/***
 *	Checker Interface Repechage V1.0
***/
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
namespace Checker {
    #define UP 0
    #define RIGHT 1
    #define DOWN 2
    #define STRAIGHT 2
    #define LEFT 3

    //#define LOGON 1
    //#define M2nxtR 1
    //#define OnRoadDEBUG 1
    //#define EndDEBUG 1
    //#define TagDEBUG 1
    //#define conflictDEBUG 1
    //#define InnerDEBUG 1
    //#define CheckReadFun 1
    //#define CheckUpdateFun 1
    using namespace std;
    typedef long long ll;

    const int maxn = 1e6 + 10;
    const int INF = 0x3f3f3f3f;
    const int waitingStatus = 1;
    const int endStatus = 2;
    unordered_map<int, int> car_map;
    unordered_map<int, int> road_map;
    unordered_map<int, int> cross_map;
    int car_index, car_hashBack[maxn];
    int road_index, road_hashBack[maxn];
    int cross_index, cross_hashBack[maxn];
    int dirHash[4][4] = {
            {-1,       LEFT,     STRAIGHT, RIGHT},
            {RIGHT, -1,          LEFT,     STRAIGHT},
            {STRAIGHT, RIGHT, -1,          LEFT},
            {LEFT,     STRAIGHT, RIGHT, -1}
    };
    int dirCheck[4][4][2] = {
            {{-1, -1}, {3,  2},  {3,  1},  {1,  2}},
            {{2,  3},  {-1, -1}, {0,  3},  {0,  2}},
            {{3,  1},  {3,  0},  {-1, -1}, {1,  0}},
            {{2,  1},  {0,  2},  {0,  1},  {-1, -1}}
    };
    int systemTime = 0;         // system scheduling time
    int TSum = 0, TSumPri = 0, lastPriCarArriveTime = 0, firstPriCarStartTime = 0;
    int carsMaxSpeed = 0, carsMinSpeed = INF, priCarMaxSpeed = 0, priCarMinSpeed = INF, priCarsCounter = 0;
    int carsFirstTime = INF, carsLastTime = 0, priCarsFirstTime = INF, priCarsLastTime = 0;
    int priCarLastArrive = 0;
    int totTimeUsed = 0;
    int totCarCounter = 0;
    int onRoadCarsCounter = 0;
    int waitingCarsCounter = 0;
    int lastWaitingCarsCounter = 0;
    int inGarageCarsCounter = 0;
    bool isDeadLock = false;
    bool isAllCarsReach = false;
    set<int> src, dst, priSrc, priDst;

    struct Car;
    struct Road;
    struct Cross;

    struct Car {
        /* Cars' argv about itself*/
        int id;
        int from;           // src cross id
        int to;             // dst cross id
        int speed;
        int status;
        int planTime;
        int RealStartTime;
        bool isPriority;
        bool isPreSet;

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

        Car(const int &_id, const int &_from, const int &_to, const int &_speed, const int &_planTime,
            const bool &_isPriority, const bool &_isPreSet) {
            id = _id;
            to = _to;
            nextCrossId = from = _from;
            speed = _speed;
            planTime = _planTime;
            status = curPathId = 0;
            isPriority = _isPriority;
            isPreSet = _isPreSet;
            RealStartTime = curRoadId = curChannelId = curPosition = -1;
        }

        bool operator<(const Car &x) const {
            if (RealStartTime != x.RealStartTime) return RealStartTime < x.RealStartTime;
            return id < x.id;
        }

        void curMessage() {
            cout << car_hashBack[id] << " " << road_hashBack[curRoadId] << " " << curChannelId << " "
                 << curPosition << " " << curPathId << " " << cross_hashBack[nextCrossId] << endl;
        }

        void output() {
            cout << car_hashBack[id] << " " << cross_hashBack[from] << " " << cross_hashBack[to]
                 << " " << speed << " " << planTime << " " << planTime << endl;
        }

        void reset() {
            nextCrossId = from;
            status = curPathId = 0;
            curRoadId = curChannelId = curPosition = -1;
        }

        int nextRoadId() {
            if (curPathId + 1 == path.size()) return -1;
            return path[curPathId + 1];
        }

        void outAnswer() {
            cout << car_hashBack[id] << " ";
            for (int i = 0; i < path.size(); ++i) {
                cout << road_hashBack[path[i]] << " ";
            }
            cout << endl;
        }

        void updateNextCrossId();

        void
        updateMessage(const int &_curRoadID, const int &_curChannelID, const int &_curPosition, const int &_curPathID);
    };

    struct TuringUnit {
        int carID, score;

        bool operator<(const TuringUnit &rhs) const;

        TuringUnit(const int &_carID, const int &_score) {
            carID = _carID;
            score = _score;
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
        vector<int> carInitList[2];
        priority_queue<TuringUnit> turing[2];

        Road() {}

        Road(const int &_id, const int &_length, const int &_speed, const int &_channelNum,
             const int &_from, const int &_to, const bool &_isDuplex) {
            id = _id;
            length = _length;
            speed = _speed;
            channelNum = _channelNum;
            from = _from;
            to = _to;
            isDuplex = _isDuplex;
            if (isDuplex) channelNum = channelNum << 1;
            /* initialization for cars' vector*/
            carInitList[0].clear();
            carInitList[1].clear();
            for (int i = 0; i < channelNum; ++i) {
                RoadCars.push_back(list<int>());
            }
        }

        int getFrontCarId(const int &channel, const int &backCarId) {
            if (RoadCars[channel].size() == 1) return -1;    // the first car
            int frontCarId = -1;
            for (auto &x : RoadCars[channel]) {
                if (x == backCarId) break;
                frontCarId = x;
            }
            return frontCarId;
        }

        list<int>::iterator removeCarFromRoadCars(const int &channelId, const int &carId) {
            for (list<int>::iterator it = RoadCars[channelId].begin(); it != RoadCars[channelId].end(); ++it) {
                if (*it == carId) {
                    return RoadCars[channelId].erase(it);
                }
            }
            //        return false;
        }

        void output() {
            cout << road_hashBack[id] << " " << length << " " << speed << " " << channelNum << " "
                 << cross_hashBack[from] << " " << cross_hashBack[to] << " " << isDuplex << endl;
        }

        void constructSchedulingUnit();

        inline void addToCarInitList(Car &curCar, const int &index) {
            carInitList[index].push_back(curCar.id);
        }

        inline void addToRoadCar(Car &curCar, const int &channelID) {
            RoadCars[channelID].push_back(curCar.id);
        }

        void arrangeOnRoad(bool isPriorityOnly, const int &index);

        int arrangeJudge(Car &, const int &);

        int calCarScore(Car &curCar);

        void driveCurrentRoad();

        void driveCurrentChannel(Cross &curCross, const int &channelID);

        bool driveCarAndTag(Car &curCar, Car &frontCar, const int &isFirst, const int &index);

        bool moveToNextRoad(Car &curCar, Cross &curCross, int & optionType);

        void forceCheck();
    };

    struct Cross {
        int id;
        int order[4];
        int connect[4]; // 0 up / 1 right / 2 down / 3 left
        unordered_map<int, int> hashPosition;

        void output() {
            cout << cross_hashBack[id] << " ";
            for (int i = 0; i < 4; ++i) cout << (connect[i] == -1 ? -1 : road_hashBack[connect[i]]) << " ";
            cout << endl;
        }

        void update() {
            hashPosition.clear();
            for (int i = 0; i < 4; ++i) {
                if (connect[i] != -1) {
                    hashPosition[connect[i]] = i;
                }
            }
        }

        inline int get_Direction(const int nextRoadId) {
            for (int i = 0; i < 4; ++i) {
                if (connect[i] == nextRoadId)
                    return i;
            }
        }

        inline int get_Road(const int &DIR) { return connect[DIR]; }

        inline int get_Position(const int &roadId) {
            return hashPosition.count(roadId) == 0 ? -1 : hashPosition[roadId];
        }

        void scheduling();

        bool conflict(Car &curCar, Road &curRoad);

        bool conflictJudge(Car &curCar, const int &dirToCheck, const int &curCarDir, const int &curCarNextDir,
                           const int &cd);
    };

    vector<Car> cars, cars_tmp;
    vector<Road> roads, roads_tmp;
    vector<Cross> crosses, crosses_tmp;

    set<int> isCarAdded;
    set<int> deadLockCrossId;
    set<int> deadLockRoadId;
    vector<vector<int> > crossOrder;
    unordered_map<int, vector<pair<int, int> > > roadOrder;

    void updateCrossOrder() {
        vector<int> &cur = crossOrder[systemTime - 1];
        for (int i = 0; i < crosses.size(); ++i) {
            Cross &curCross = crosses[i];
            int tot = 0;
            for (int j = 0; j < 4; ++j) {
                if (curCross.connect[j] == -1) continue;
                else {
                    Road &curRoad = roads[curCross.connect[j]];
                    if (curRoad.to == curCross.id) tot += curRoad.turing[0].size();
                    else tot += curRoad.turing[1].size();
                }
            }
            cur.push_back(tot);
        }
    }

    void updateRoadOrder(const int carId, const int roadId, bool isin) {
        auto &curVec = roadOrder[carId];
        if (isin) {
            assert(int(curVec.size()) % 2 == 0);
            curVec.push_back(make_pair(roadId, systemTime));
        } else {
            assert(int(curVec.size()) % 2 == 1);
            curVec.push_back(make_pair(roadId, systemTime));
        }
    }

    void UpdateMessage(Car &curCar);

    void Car::updateNextCrossId() {
        nextCrossId =
                roads[path[curPathId]].from == nextCrossId ? roads[path[curPathId]].to : roads[path[curPathId]].from;
    }

    void Car::updateMessage(const int &_curRoadID, const int &_curChannelID, const int &_curPosition,
                            const int &_curPathID) {
        curRoadId = _curRoadID;
        curChannelId = _curChannelID;
        curPosition = _curPosition;
        curPathId = _curPathID;
        status = endStatus;
        updateNextCrossId();
    }

    int Road::calCarScore(Car &curCar) {
        if (isDuplex) {
            int offset = (curCar.curChannelId >= channelNum / 2) ? (channelNum / 2) : 0;
            return (curCar.curPosition - 1) * (channelNum / 2) + (curCar.curChannelId - offset);
        } else {
            return (curCar.curPosition - 1) * channelNum + curCar.curChannelId;
        }
    }

    bool Road::moveToNextRoad(Car &curCar, Cross &curCross, int & optionType) {

        if (curCar.curPathId + 1 == curCar.path.size()) {
            curCar.status = endStatus;
            totTimeUsed += systemTime - curCar.planTime;
            --onRoadCarsCounter;
            --waitingCarsCounter;
            ++inGarageCarsCounter;
            updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curCar.path[curCar.curPathId]], false);
            removeCarFromRoadCars(curCar.curChannelId, curCar.id);
#ifdef EndDEBUG
            cout << "[End] carID: " << car_hashBack[curCar.id] << endl;
#endif
            UpdateMessage(curCar);
            return true;
        } else {
            Road &nextRoad = roads[curCar.nextRoadId()];
            int st = -1, ed = -1, moveDistance = min(curCar.speed, nextRoad.speed) - (curCar.curPosition - 1);
            int carPathID = curCar.curPathId, carChannelID = curCar.curChannelId, carRoadID = curCar.curRoadId, carPosition = curCar.curPosition;
            if (moveDistance <= 0) {
                carPosition = 1;
#ifdef M2nxtR
                cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << " Road " << road_hashBack[id]
                     << " move to Pos " << carPosition << endl;
#endif
                curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                --waitingCarsCounter;
                return true;
            }
            if (nextRoad.isDuplex)
                if (nextRoad.from == curCross.id) st = 0, ed = nextRoad.channelNum / 2;
                else st = nextRoad.channelNum / 2, ed = nextRoad.channelNum;
            else st = 0, ed = nextRoad.channelNum;
            for (int curChannelID = st; curChannelID != ed; ++curChannelID) {
                if (nextRoad.RoadCars[curChannelID].empty()) {
                    removeCarFromRoadCars(curCar.curChannelId, curCar.id);
                    carRoadID = nextRoad.id;
                    carPosition = nextRoad.length - moveDistance + 1;
                    carChannelID = curChannelID;
                    ++carPathID;
#ifdef M2nxtR
                    cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << ", from Road " << road_hashBack[id]
                        << " Pos " << curCar.curPosition << " to Road " << road_hashBack[carRoadID] << " Pos " << carPosition << endl;
#endif
                    curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                    nextRoad.addToRoadCar(curCar, carChannelID);
                    --waitingCarsCounter;
                    optionType = 1;
                    return true;
                } else {
                    Car &lastCar = cars[nextRoad.RoadCars[curChannelID].back()];
                    if (moveDistance >= nextRoad.length - lastCar.curPosition + 1) {
                        if (lastCar.status == waitingStatus) {
#ifdef M2nxtR
                            cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << ", on Road " << road_hashBack[id]
                                 << " WAIT, cause frontCarID" << car_hashBack[lastCar.id] << endl;
#endif
                            return false;
                        } else {
                            if (lastCar.curPosition == nextRoad.length) continue;
                            else {
                                removeCarFromRoadCars(curCar.curChannelId, curCar.id);
                                carRoadID = nextRoad.id;
                                carPosition = lastCar.curPosition + 1;
                                carChannelID = curChannelID;
                                ++carPathID;
#ifdef M2nxtR
                                cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << ", from Road " << road_hashBack[id]
                                     << " Pos " << curCar.curPosition << " to Road " << road_hashBack[carRoadID] << " Pos " << carPosition << endl;
#endif
                                curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                                nextRoad.addToRoadCar(curCar, carChannelID);
                                --waitingCarsCounter;
                                optionType = 1;
                                return true;
                            }
                        }
                    } else {
                        removeCarFromRoadCars(curCar.curChannelId, curCar.id);
                        carRoadID = nextRoad.id;
                        carPosition = nextRoad.length - moveDistance + 1;
                        carChannelID = curChannelID;
                        ++carPathID;
#ifdef M2nxtR
                        cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << ", from Road " << road_hashBack[id]
                             << " Pos " << curCar.curPosition << " to Road " << road_hashBack[carRoadID] << " Pos " << carPosition << endl;
#endif
                        curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                        nextRoad.addToRoadCar(curCar, carChannelID);
                        --waitingCarsCounter;
                        optionType = 1;
                        return true;
                    }
                }
            }
            carPosition = 1;
#ifdef M2nxtR
            cout << "[M2nxtR] carID " << car_hashBack[curCar.id] << " Road " << road_hashBack[id]
                 << " move to Pos " << carPosition << endl;
#endif
            curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
            --waitingCarsCounter;
            return true;
        }
    }

    int Road::arrangeJudge(Car &curCar, const int &curChannelID) {
        /**
         * @brief
         * @param curCar
         * @param curChannelID
         *
         * @return
         *     <-1> next channelID is full
         *     <0>  wait
         *     <1>  arrange curCar to its place
         */
        if (RoadCars[curChannelID].empty()) {
            ++onRoadCarsCounter;
            curCar.updateMessage(id, curChannelID, length - min(curCar.speed, speed) + 1, 0);
            RoadCars[curChannelID].push_back(curCar.id);
            return 1;
        } else {
            Car &frontCar = cars[RoadCars[curChannelID].back()];
            if (min(curCar.speed, speed) >= length - frontCar.curPosition + 1) {
                if (frontCar.status == endStatus) {
                    if (frontCar.curPosition == length)
                        return -1;
                    ++onRoadCarsCounter;
                    curCar.updateMessage(id, curChannelID, frontCar.curPosition + 1, 0);
                    RoadCars[curChannelID].push_back(curCar.id);
                    return 1;
                } else
                    return 0;
            } else {
                ++onRoadCarsCounter;
                curCar.updateMessage(id, curChannelID, length - min(curCar.speed, speed) + 1, 0);
                RoadCars[curChannelID].push_back(curCar.id);
                return 1;
            }
        }
    }

    void Road::arrangeOnRoad(bool isPriorityOnly, const int &index) {
        for (auto curCarID = carInitList[index].begin(); curCarID != carInitList[index].end();) {
            if (isPriorityOnly) {
                if (cars[*curCarID].isPriority && cars[*curCarID].RealStartTime > systemTime) return;
                if (!cars[*curCarID].isPriority) return;
            } else {
                if (cars[*curCarID].isPriority && cars[*curCarID].RealStartTime > systemTime) {
                    ++curCarID;
                    continue;
                }
                if (!cars[*curCarID].isPriority && cars[*curCarID].RealStartTime > systemTime) return;
            }
            int st = 0, ed = 0;
            if (isDuplex)
                if (index == 0) st = 0, ed = channelNum / 2;
                else st = channelNum / 2, ed = channelNum;
            else st = 0, ed = channelNum;
            int returnValue = -1;
            for (int curChannelID = st; curChannelID != ed; ++curChannelID) {
                returnValue = arrangeJudge(cars[*curCarID], curChannelID);
                if (returnValue == 1) {
#ifdef OnRoadDEBUG
                    cout << "[OnRoad] carID: " << car_hashBack[cars[*curCarID].id] << " roadID: "
                        << road_hashBack[cars[*curCarID].curRoadId] << " chan: " << cars[*curCarID].curChannelId
                        << " pos: " << cars[*curCarID].curPosition << endl;
#endif
                    updateRoadOrder(car_hashBack[*curCarID], road_hashBack[cars[*curCarID].curRoadId], true);
                    curCarID = carInitList[index].erase(curCarID);
                    break;
                } else if (returnValue == 0) {
                    break;
                } else if (returnValue == -1) continue;
            }
            if (returnValue == 0 || returnValue == -1)
                ++curCarID;

        }
    }

    bool Road::driveCarAndTag(Car &curCar, Car &frontCar, const int &isFirst, const int &index) {
        bool returnValue = true;
        //    curCar.curMessage();
        if (isFirst) {
            if (curCar.curPosition - 1 < min(curCar.speed, speed)) {
                curCar.status = waitingStatus;
                turing[index].push(TuringUnit(curCar.id, calCarScore(curCar)));
                returnValue = false;
#ifdef TagDEBUG
                cout << "[Tags] CarID " << car_hashBack[curCar.id] << ", RoadID " << road_hashBack[curCar.curRoadId]
                     << ", Chan " << curCar.curChannelId <<", Pos " << curCar.curPosition << ", tag WAIT cause TURING" << endl;
#endif
                //                return false;
                //            }
            } else {
                curCar.curPosition -= min(curCar.speed, speed);
                curCar.status = endStatus;
                //            return true;
            }
        } else {
            if (curCar.curPosition - frontCar.curPosition <= min(curCar.speed, speed)) {
                if (frontCar.status == waitingStatus) {
                    curCar.status = waitingStatus;
                    returnValue = false;
#ifdef TagDEBUG
                    cout << "[Tags] CarID " << car_hashBack[curCar.id] << ", RoadID " << road_hashBack[curCar.curRoadId]
                         << ", Chan " << curCar.curChannelId << ", Pos " << curCar.curPosition << ", tag WAIT cause FRONTCAR WAIT" << endl;
#endif
                    //                return false;
                } else {
                    curCar.curPosition = frontCar.curPosition + 1;
                    curCar.status = endStatus;
                    //                return true;
                }
            } else {
                curCar.curPosition -= min(curCar.speed, speed);
                curCar.status = endStatus;
                //            return true;
            }
        }
        if (returnValue) {
#ifdef TagDEBUG
            cout << "[Tags] CarID " << car_hashBack[curCar.id] << ", RoadID " << road_hashBack[curCar.curRoadId]
                 << ", Chan " << curCar.curChannelId << ", move to Pos " << curCar.curPosition << ", tag END" << endl;
#endif
        } else ++waitingCarsCounter;
        return returnValue;
    }

    void Road::driveCurrentChannel(Cross &curCross, const int &channelID) {
        int carPathID, carChannelID, carRoadID, carPosition;
        if (RoadCars[channelID].empty()) return;
        list<int>::iterator curCarID = RoadCars[channelID].begin(), frontCarID = RoadCars[channelID].begin();
        for (; curCarID != RoadCars[channelID].end(); ++curCarID) {
            Car &curCar = cars[*curCarID];
            Car &frontCar = cars[*frontCarID];
            //        cout << car_hashBack[curCar.id] << " " << car_hashBack[frontCar.id] << " " << endl;
            carPathID = curCar.curPathId, carChannelID = curCar.curChannelId, carRoadID = curCar.curRoadId, carPosition = curCar.curPosition;
            if (curCar.status == waitingStatus) {
                if (frontCar.status == waitingStatus) {
                    if (curCar.curPosition - 1 < min(curCar.speed, speed)) {
                        int index = to == curCross.id ? 0 : 1;
                        turing[index].push(TuringUnit(curCar.id, calCarScore(curCar)));
#ifdef InnerDEBUG
                        cout << "[Inner] CarID " << car_hashBack[curCar.id] << ", Road " << road_hashBack[id]
                             << " Turing, return. " << endl;
#endif
                        return;
                    } else {
                        carPosition -= min(curCar.speed, speed);
#ifdef InnerDEBUG
                        cout << "[Inner] CarID " << car_hashBack[curCar.id] << ", Road " << road_hashBack[id]
                             << ", move to Pos " << carPosition << endl;
#endif
                        curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                        --waitingCarsCounter;
                    }
                } else {
                    int moveDis = min(min(curCar.speed, speed), curCar.curPosition - frontCar.curPosition - 1);
                    carPosition -= moveDis;
#ifdef InnerDEBUG
                    cout << "[Inner] CarID " << car_hashBack[curCar.id] << ", Road " << road_hashBack[id]
                         << ", move to Pos " << carPosition << endl;
#endif
                    curCar.updateMessage(carRoadID, carChannelID, carPosition, carPathID);
                    --waitingCarsCounter;
                }
            } else {
                if (curCarID == RoadCars[channelID].begin()) continue;
                else break;
            }
            frontCarID = curCarID;
        }
    }

    void Road::driveCurrentRoad() {
        list<int>::iterator it, frontIt;
        //    cout << "curRoad: " << road_hashBack[id] << endl;
        if (isDuplex) {
            for (int curChannelID = 0; curChannelID < channelNum / 2; ++curChannelID) {
                if (RoadCars[curChannelID].empty()) continue;
                bool isFirst = true;
                frontIt = RoadCars[curChannelID].begin();
                for (it = RoadCars[curChannelID].begin(); it != RoadCars[curChannelID].end(); ++it) {
                    if (isFirst) {
                        driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 0);
                        isFirst = false;
                    } else driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 0);
                    frontIt = it;
                }
            }
            for (int curChannelID = channelNum / 2; curChannelID < channelNum; ++curChannelID) {
                if (RoadCars[curChannelID].empty()) continue;
                bool isFirst = true;
                frontIt = RoadCars[curChannelID].begin();
                for (it = RoadCars[curChannelID].begin(); it != RoadCars[curChannelID].end(); ++it) {
                    if (isFirst) {
                        driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 1);
                        isFirst = false;
                    } else driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 1);
                    frontIt = it;
                }
            }
        } else {
            for (int curChannelID = 0; curChannelID < channelNum; ++curChannelID) {
                if (RoadCars[curChannelID].empty()) continue;
                bool isFirst = true;
                frontIt = RoadCars[curChannelID].begin();
                for (it = RoadCars[curChannelID].begin(); it != RoadCars[curChannelID].end(); ++it) {
                    if (isFirst) {
                        driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 0);
                        isFirst = false;
                    } else driveCarAndTag(cars[*it], cars[*frontIt], isFirst, 0);
                    frontIt = it;
                }
            }
        }
    }

    void Road::forceCheck() {
        bool isError = false;
        if (!turing[0].empty() && !turing[1].empty()) {
            cout << " turing Error! RoadID:" << road_hashBack[id] << " turing NOT empty!" << endl;
            if (!turing[0].empty()) {
                cout << "truing[0]: " << endl;
                while (!turing[0].empty()) {
                    TuringUnit it = turing[0].top();
                    turing[0].pop();
                    cout << car_hashBack[it.carID] << endl;
                }
            }
            if (!turing[1].empty()) {
                cout << "truing[1]: " << endl;
                while (!turing[1].empty()) {
                    TuringUnit it = turing[1].top();
                    turing[1].pop();
                    cout << car_hashBack[it.carID] << endl;
                }
            }
            isError = true;
        }

        for (auto &channel:RoadCars) {
            if (RoadCars.empty()) continue;
            auto back = channel.begin(), front = channel.begin();
            for (; back != channel.end(); ++back) {
                if (cars[*back].status == waitingStatus) {
                    isError = true;
                    cout << "Status Error! RoadID: " << road_hashBack[id] << "carID " << car_hashBack[*back]
                         << " ChannelID " << cars[*back].curChannelId << endl;
                }
                if (cars[*back].curPosition <= 0 || cars[*back].curPosition > length) {
                    isError = true;
                    cout << "Position out of bounds! Error! RoadID: " << road_hashBack[id] << "carID "
                         << car_hashBack[*back]
                         << " ChannelID " << cars[*back].curChannelId << " Position " << cars[*back].curPosition
                         << endl;
                }
                if (*back != *front) {
                    if (cars[*back].curPosition <= cars[*front].curPosition) {
                        isError = true;
                        cout << "Position Error! RoadID: " << road_hashBack[id] << " frontCarID "
                             << car_hashBack[*front]
                             << " Position " << cars[*front].curPosition << " backCarID " << car_hashBack[*back]
                             << " Position " << cars[*back].curPosition << endl;
                    }
                }
                front = back;
            }
        }

        if (isError)
            exit(2420);
    }

    bool TuringUnit::operator<(const TuringUnit &rhs) const {
        if (cars[carID].isPriority == cars[rhs.carID].isPriority) return score > rhs.score;
        return cars[carID].isPriority < cars[rhs.carID].isPriority;
    }

    bool Cross::conflictJudge(Car &curCar, const int &dirToCheck, const int &curCarDir, const int &curCarNextDir,
                              const int &cd) {
        if (connect[dirToCheck] == -1) return false;
        else {
            const Road &targetRoad = roads[connect[dirToCheck]];
            int index = id == targetRoad.to ? 0 : 1;
            if (targetRoad.turing[index].empty()) {
                return false;
            } else {
                Car &targetRoadFirstCar = cars[targetRoad.turing[index].top().carID];
                int dir = -1, curDir = -1, nextDir = -1;
                curDir = get_Direction(targetRoadFirstCar.curRoadId);
                if (targetRoadFirstCar.curPathId + 1 == targetRoadFirstCar.path.size()) {
                    nextDir = (curDir + 2) % 4;
                    dir = STRAIGHT;
                } else {
                    const Road &nextRoad = roads[targetRoadFirstCar.nextRoadId()];
                    nextDir = get_Direction(nextRoad.id);
                    dir = dirHash[curDir][nextDir];
                }
                if (curCarNextDir != nextDir) return false;
                if (curCar.isPriority > targetRoadFirstCar.isPriority) return false;
                else if (curCar.isPriority < targetRoadFirstCar.isPriority) return true;
                else {
                    if (curCarDir == STRAIGHT) return false;
                    else if (curCarDir == LEFT) return dir == STRAIGHT;
                    else if (curCarDir == RIGHT) return true;
                }
            }
        }
    }

    bool Cross::conflict(Car &curCar, Road &curRoad) {
        int dir = -1, dirToCheck = -1, curDir = -1, nextDir = -1;
        if (curCar.curPathId + 1 == curCar.path.size()) {
            curDir = get_Direction(curRoad.id);
            nextDir = (curDir + 2) % 4;
            dir = STRAIGHT;
        } else {
            Road &nextRoad = roads[curCar.nextRoadId()];
            curDir = get_Direction(curRoad.id), nextDir = get_Direction(nextRoad.id);
            dir = dirHash[curDir][nextDir];
        }
        assert(dir >= 1 && dir <= 3);
        string tips = "";
        if (dir == STRAIGHT) tips = "STRAIGHT";
        else if (dir == LEFT) tips = "LEFT";
        else if (dir == RIGHT) tips = "RIGHT";

        for (int cd = 0; cd < 2; ++cd) {
            dirToCheck = dirCheck[curDir][nextDir][cd];
            if (dirToCheck == -1) break;
            if (conflictJudge(curCar, dirToCheck, dir, nextDir, cd)) {
#ifdef conflictDEBUG
                cout << "[Conflict] CarID " << car_hashBack[curCar.id] << " go " << tips << " failed,"
                     << endl;
#endif
                return true;
            }
        }
#ifdef conflictDEBUG
        cout << "[NConflict] CarID " << car_hashBack[curCar.id] << " go " << tips << endl;
#endif
        return false;

    }

    void Cross::scheduling() {
//        cout << "curCross" << cross_hashBack[id] << endl;
        for (int i = 0; i < 4; ++i) {
            if (order[i] == -1) continue;
            else {
                Road &curRoad = roads[order[i]];
//                cout << "curRoad" << road_hashBack[curRoad.id] << endl;
                int index = curRoad.to == id ? 0 : 1;
                while (!curRoad.turing[index].empty()) {
                    Car &curCar = cars[curRoad.turing[index].top().carID];
                    int curChannelID = curCar.curChannelId;
                    if (conflict(curCar, curRoad)) break;
                    int optionType = 0;
                    if (curRoad.moveToNextRoad(curCar, *this, optionType)) {
//                        cout << "test1" << endl;
//                        cout << curCar.curPathId << endl;
                        if(optionType == 1) {
                            updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curCar.path[curCar.curPathId - 1]], false);
                            updateRoadOrder(car_hashBack[curCar.id], road_hashBack[curCar.path[curCar.curPathId]], true);
                        }
//                        cout << "test2" << endl;
                        curRoad.turing[index].pop();
                        curRoad.driveCurrentChannel(*this, curChannelID);
                        if(curRoad.isDuplex || id == curRoad.to) {
                            int initListID = id == curRoad.to ? 0 : 1;
                            if(!curRoad.carInitList[initListID].empty())
                                curRoad.arrangeOnRoad(true, index);
                        }
                    } else break;
                }
            }
        }
    }

    inline bool cmp_carHash(const Car &C1, const Car &C2) { return C1.id < C2.id; }

    inline bool cmp_roadHash(const Road &R1, const Road &R2) { return R1.id < R2.id; }

    inline bool cmp_crossHash(const Cross &C1, const Cross &C2) { return C1.id < C2.id; }

    inline bool cmp_carInitList(int &C1, int &C2) {
        if (cars[C1].isPriority == cars[C2].isPriority)
            if (cars[C1].RealStartTime == cars[C2].RealStartTime) return cars[C1].id < cars[C2].id;
            else return cars[C1].RealStartTime < cars[C2].RealStartTime;
        return cars[C1].isPriority > cars[C2].isPriority;
    }

    void read_car(const string &path) {
        ifstream in(path);
        string info;
        char ch;
        int id, from, to, speed, planTime, isPriority, isPreSet;
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
            car_stream >> ch;
            car_stream >> isPriority;
            car_stream >> ch;
            car_stream >> isPreSet;
            cars_tmp.push_back(Car(id, from, to, speed, planTime, isPriority, isPreSet));
        }
        sort(cars_tmp.begin(), cars_tmp.end(), cmp_carHash);
        car_index = 0;
        for (auto &x : cars_tmp) {
            car_map[x.id] = car_index;
            car_hashBack[car_index] = x.id;
            x.id = car_index++;
            cars.push_back(x);
        }
        cars_tmp.clear();
    }

    void read_road(const string &path) {
        ifstream in(path);
        string info;
        road_map[-1] = -1;
        char ch;
        int id, length, speed, channelnum, from, to;
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
        for (auto &x : roads_tmp) {
            road_map[x.id] = road_index;
            road_hashBack[road_index] = x.id;
            x.id = road_index++;
            roads.push_back(x);
        }
        roads_tmp.clear();
    }

    void read_cross(const string &path) {
        ifstream in(path);
        string info;
        char ch;
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
        for (auto &x : crosses_tmp) {
            cross_map[x.id] = cross_index;
            cross_hashBack[cross_index] = x.id;
            x.id = cross_index++;
            crosses.push_back(x);
        }
        crosses_tmp.clear();
    }

    void read_presetAnswer(const string &path) {
        ifstream in(path);
        string info;
        char ch;
        getline(in, info);          // ignore the first line of input
        int tmp_roadId, tmp_carId, tmp_startTime, carID;
        while (getline(in, info)) {
            stringstream answer_stream(info);
            answer_stream >> ch;            // scan '('
            answer_stream >> tmp_carId;
            answer_stream >> ch;            // scan ','
            answer_stream >> tmp_startTime;
            carID = car_map[tmp_carId];
            cars[carID].RealStartTime = tmp_startTime;
            cars[carID].path.clear();
            while (true) {
                answer_stream >> ch;        // scan ',' or ')'
                if (ch == ')') break;        // break when scan ')'
                answer_stream >> tmp_roadId;
                cars[carID].path.push_back(road_map[tmp_roadId]);
            }
        }
    }

    void update_RoadCar_makeMap() {
        // [Done] update Road [from, to] to hash &  update Car [from, to] to hash
        for (auto &x:cars) {
            x.from = cross_map[x.from];
            x.to = cross_map[x.to];
            x.nextCrossId = cross_map[x.nextCrossId];
        }
        for (auto &x:roads) {
            x.from = cross_map[x.from];
            x.to = cross_map[x.to];
        }
    }

    void UpdateMessage(Car &curCar) {
        TSum += systemTime - curCar.planTime;
        if (curCar.isPriority) {
            TSumPri += systemTime - curCar.planTime;
            priCarLastArrive = max(priCarLastArrive, systemTime);
        }
    }

    void deadLockMessages() {
        deadLockCrossId.clear();
        deadLockRoadId.clear();
        for (auto &cross : crosses) {
            for (int i = 0; i < 4; ++i) {
                if (cross.order[i] == -1) continue;
                auto &road = roads[cross.order[i]];
                int index = road.to == cross.id ? 0 : 1;
                if (!road.turing[index].empty()) {
                    deadLockCrossId.insert(cross_hashBack[cross.id]);
                    deadLockRoadId.insert(road_hashBack[road.id]);
                }

            }
        }
    }

    void modifyCar(int carId, const int &realStarTime, vector<int> &path) {
        carId = car_map[carId];
        isCarAdded.insert(carId);
        Car &curCar = cars[carId];
        curCar.RealStartTime = realStarTime;
        curCar.path.clear();
        for (int i = 0; i < path.size(); ++i)
            curCar.path.push_back(road_map[path[i]]);
    }

    void reset() {

        if (isDeadLock) {
            for (auto &road:roads) {
                while (!road.turing[0].empty()) road.turing[0].pop();
                while (!road.turing[1].empty()) road.turing[1].pop();
                road.carInitList[0].clear();
                road.carInitList[1].clear();
                for (auto &channel:road.RoadCars) channel.clear();
            }
        }
        for (auto &x : crossOrder) x.clear();
        for (auto &x : roadOrder) x.second.clear();
        for (auto &car : cars) {
            car.reset();
            roadOrder[car_hashBack[car.id]] = vector<pair<int, int> >();
        }
        totCarCounter = isCarAdded.size();
        systemTime = inGarageCarsCounter = 0;
        onRoadCarsCounter = lastWaitingCarsCounter = waitingCarsCounter = 0;
        isAllCarsReach = isDeadLock = false;
        for (auto &curCar : cars) {
            if (isCarAdded.count(curCar.id)) {
                Road &firstRoad = roads[curCar.path.front()];
                int index = firstRoad.from == curCar.from ? 0 : 1;
                firstRoad.addToCarInitList(curCar, index);
            }
        }
        for (auto &curRoad : roads) {
            if (!curRoad.carInitList[0].empty())
                sort(curRoad.carInitList[0].begin(), curRoad.carInitList[0].end(), cmp_carInitList);
            if (!curRoad.carInitList[1].empty())
                sort(curRoad.carInitList[1].begin(), curRoad.carInitList[1].end(), cmp_carInitList);
        }
    }

    void run() {

        crossOrder.push_back(vector<int>());
        while (true) {
            ++systemTime;
            //cout << systemTime << endl;
            crossOrder.push_back(vector<int>());
            for (auto &road:roads) {
                road.driveCurrentRoad();
            }
            for (auto &road:roads) {
                road.arrangeOnRoad(true, 0);
                road.arrangeOnRoad(true, 1);
            }
            lastWaitingCarsCounter = waitingCarsCounter;
            updateCrossOrder();

            while (true) {
                if (waitingCarsCounter == 0) break;
                for (auto &cross:crosses)
                    cross.scheduling();
                if (lastWaitingCarsCounter == waitingCarsCounter) {
                    isDeadLock = true;
                    break;
                } else lastWaitingCarsCounter = waitingCarsCounter;
            }

            if (isDeadLock) {
                cout << "DeadLock" << endl;
                deadLockMessages();
                break;
            }

            //        for(auto & road:roads) road.forceCheck();
            for (auto &road:roads) {
                road.arrangeOnRoad(false, 0);
                road.arrangeOnRoad(false, 1);
            }
            if (inGarageCarsCounter == totCarCounter) {
                isAllCarsReach = true;
                break;
            }
        }
    }
}

//read_car("car.txt");
//read_road("road.txt");
//read_cross("cross.txt");
//read_presetAnswer("presetAnswer.txt");
//update_RoadCar_makeMap();
//reset()
//run()
//modifycar()

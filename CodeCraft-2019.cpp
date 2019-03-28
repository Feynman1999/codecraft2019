#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <algorithm>
#include <queue>
#include <time.h>
#include <assert.h>
#include <cmath>
#include <map>
#define mp make_pair
using namespace std;
const double inf=1e17;
const int GO_STRAIGHT=0;
const int TURN_LEFT=1;
const int TURN_RIGHT=2;
const double LEFT_PENALTY=2;
const double RIGHT_PENALTY=4;
const double Cross_penalty_alpha=20.0;
const int Lucky_number=535;
const double Time_reduction_rate=1.0;
map<int,int> road_id_to_index;
struct Road
{
    int id;
    int length;
    int speed;
    int channel;
    int from;
    int to;
    bool isDuplex;
    vector<int> mi;
    void set(int l,int r,int v)
    {
        int last=mi.size();
        if(last<=r)
        {
            mi.resize(r+1);
            for(int i=last;i<=r;++i) mi[i]=speed;
        }
        for(int i=l;i<=r;++i) mi[i]=min(mi[i],v);
    }
    int query(int l)
    {
        if(l>=mi.size()) return speed;
        return mi[l];
    }
    void output()
    {
        cout<<id<<" "<<length<<" "<<speed<<" "<<channel<<" "<<from<<" "<<to<<" "<<isDuplex<<endl;
    }
};
vector<Road> road;
struct Car
{
    int id;
    int from;
    int to;
    int speed;
    int initTime;
    int planTime;
    double value;
    void output()
    {
        cout<<id<<" "<<from<<" "<<to<<" "<<speed<<" "<<planTime<<endl;
    }
};
vector<Car> car;
bool cmp1(const Car &a,const Car &b)
{
    if(a.speed!=b.speed) return a.speed<b.speed;
    return a.planTime<b.planTime;
}
bool cmp_value(const Car &a,const Car &b)
{
    return a.value>b.value;
}
struct Cross
{
    int id;
    int dir[4];
    int car_start_from_this;
    vector<int> block;
    void set(int t)
    {
        int l=max(0,t-2);
        int r=t+2;
        int last=block.size();
        if(last<=r)
        {
            block.resize(r+1);
            for(int i=last;i<=r;++i) block[i]=0;
        }
        for(int i=l;i<=r;++i) block[i]+=1;
    }
    int query_car_num(int t)
    {
        int l=max(0,t-2);
        int r=min(t+2,int(block.size())-1);
        if(l>r) return 0;
        int sum=0;
        for(int i=l;i<=r;++i) sum+=block[i];
        sum=round(1.0*sum/(r-l+1));
        return sum;
    }
    double query_penalty(int t)
    {
        /*if(query_car_num(t)<=get_road_num()) return 0;
        double num=Cross_penalty_alpha*query_car_num(t)/get_road_num();
        //if(num<=get_road_num()*2) num=0;
        return num;*/

        if(query_car_num(t)<=get_road_num()*2) return 0;
        double num=query_car_num(t)*get_road_num();
        return num;
    }
    int get_road_num()
    {
        int s=0;
        for(int i=0;i<4;++i) if(dir[i]!=-1) ++s;
        return s;
    }
    void output()
    {
        cout<<id<<" "<<dir[0]<<" "<<dir[1]<<" "<<dir[2]<<" "<<dir[3]<<endl;
    }
};
vector<vector<pair<int,int>>> g;
//vector<Car> car;
int T;
//vector<Road> road;
int m;
vector<Cross> cross;
int n;
void addedge(int from,int to,int id)
{
    //cout<<from<<" "<<to<<" "<<id;
    g[from].push_back(mp(to,id));
}
void readcar(string path)
{
    ifstream in(path);
    string info;
    getline(in,info);
    car.clear();
    car.push_back({0,0,0,0,0,0,0.0});
    while(getline(in,info))
    {
        ++T;
        //cout<<tmp<<endl;
        char ch;
        Car tmp;
        stringstream s(info);
        s>>ch;
        s>>tmp.id;
        s>>ch;
        s>>tmp.from;
        s>>ch;
        s>>tmp.to;
        s>>ch;
        s>>tmp.speed;
        s>>ch;
        s>>tmp.initTime;
        tmp.planTime=tmp.initTime;
        car.push_back(tmp);
    }
    //for(auto x:car) x.output();
}
void readroad(string path)
{
    //cout<<n<<endl;
    ifstream in(path);
    string info;
    getline(in,info);
    road.clear();
    road.push_back({0,0,0,0,0,0,0});
    while(getline(in,info))
    {
        ++m;
        //cout<<info<<endl;
        char ch;
        Road tmp;
        stringstream s(info);
        s>>ch;
        s>>tmp.id;
        s>>ch;
        s>>tmp.length;
        s>>ch;
        s>>tmp.speed;
        s>>ch;
        s>>tmp.channel;
        s>>ch;
        s>>tmp.from;
        s>>ch;
        s>>tmp.to;
        s>>ch;
        s>>tmp.isDuplex;
        road.push_back(tmp);
        road_id_to_index[tmp.id]=m;
        //cout<<tmp.from<<" "<<tmp.to<<" "<<m<<end;
        addedge(tmp.from,tmp.to,m);
        if(tmp.isDuplex) addedge(tmp.to,tmp.from,m);
    }
    //for(auto x:road) x.output();
}
void readcross(string path)
{
    ifstream in(path);
    string info;
    getline(in,info);
    cross.clear();
    cross.push_back({0,0,0,0,0});
    while(getline(in,info))
    {
        ++n;
        //cout<<tmp<<endl;
        char ch;
        Cross tmp;
        stringstream s(info);
        s>>ch;
        s>>tmp.id;
        s>>ch;
        s>>tmp.dir[0];
        s>>ch;
        s>>tmp.dir[1];
        s>>ch;
        s>>tmp.dir[2];
        s>>ch;
        s>>tmp.dir[3];
        s>>ch;
        cross.push_back(tmp);
    }
    g.resize(n+1);
    for(int i=0;i<=n;++i) g[i].clear();
    //for(auto x:cross) x.output();
}
vector<bool> done;
vector<double> d,actual_d;
vector<pair<int,int>> pre;   //<pre_cross,through_road>
void dijkstra_init()
{
    done.resize(n+1);
    d.resize(n+1);
    actual_d.resize(n+1);
    pre.resize(n+1);
}
priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> q;

int checkdir(int u,int id,int v)
{
    int a=-1,b=-1;
    for(int i=0;i<4;++i)
    {
        if(cross[id].dir[i]==u) a=i;
        if(cross[id].dir[i]==v) b=i;
    }
    if(a==-1||b==-1) return GO_STRAIGHT;
    if((a+2)%4==b) return GO_STRAIGHT;
    if((a+1)%4==b) return TURN_LEFT;
    return TURN_RIGHT;
}
double cal_value(int S,int T,int speed,int start_time)
{
    //cout<<S<<" "<<T<<" "<<speed<<" "<<start_time<<endl;
    for(int i=0;i<=n;++i) d[i]=inf,done[i]=0;
    d[S]=0;
    while(!q.empty()) q.pop();
    q.push(mp(0,S));
    while(!q.empty())
    {
        pair<double,int> now=q.top();
        q.pop();
        int u=now.second;
        if(done[u]) continue;
        //cout<<u<<endl;
        done[u]=1;
        for(auto x:g[u])
        {
            int v=x.first;
            Road edge=road[x.second];
            assert(edge.speed>0);
            assert(speed>0);
            double data=now.first+1.0*edge.length/min(edge.speed,speed);
            if(data<d[v])
            {
                d[v]=data;
                q.push(mp(d[v],v));
            }
        }
    }
    return d[T];
}
vector<int> dijkstra(int S,int T,int speed,int start_time)
{
    //cout<<S<<endl;
    for(int i=0;i<=n;++i) d[i]=actual_d[i]=inf,pre[i]=mp(0,0),done[i]=0;
    d[S]=0;
    actual_d[S]=d[S];
    cross[S].set(start_time+round(d[S]));
    while(!q.empty()) q.pop();
    q.push(mp(d[S],S));
    while(!q.empty())
    {
        pair<double,int> now=q.top();
        q.pop();
        int u=now.second;
        if(done[u]) continue;
        //cout<<u<<endl;
        done[u]=1;
        for(auto x:g[u])
        {
            int v=x.first;
            Road edge=road[x.second];
            double time_in_road=1.0*edge.length/min(edge.query(start_time+round(Time_reduction_rate*actual_d[u])),speed);
            double data=now.first+time_in_road+cross[v].query_penalty(start_time+round(Time_reduction_rate*actual_d[u]+time_in_road));
            int dir=checkdir(road[pre[u].second].id,u,edge.id);
            if(dir==TURN_LEFT) data+=LEFT_PENALTY;
            if(dir==TURN_RIGHT) data+=RIGHT_PENALTY;
            if(data<d[v])
            {
                pre[v]=mp(u,x.second);
                d[v]=data;
                actual_d[v]=actual_d[u]+time_in_road;
                q.push(mp(d[v],v));
            }
        }
    }
    vector<int> ans;
    //cout<<d[T]<<endl;
    ans.clear();
    int now=T;
    while(now!=S)
    {
        ans.push_back(road[pre[now].second].id);
        road[pre[now].second].set(start_time+round(Time_reduction_rate*actual_d[pre[now].first]),start_time+round(Time_reduction_rate*actual_d[now]),speed);
        cross[now].set(start_time+round(Time_reduction_rate*actual_d[now]));
        now=pre[now].first;
    }
    cross[now].set(start_time+round(Time_reduction_rate*actual_d[now]));
    return ans;
}
bool cmp_arange_time(const int &x,const int &y)
{
    return car[x].value>car[y].value;
}
void random_add_planTime(int MOD)
{
    vector<vector<int>> tmp;
    tmp.resize(n+1);
    for(int i=1;i<=n;++i) tmp[i].clear();
    for(int i=1;i<=T;++i) tmp[car[i].from].push_back(i);
    for(int i=1;i<=n;++i)
    {
        int num=tmp[i].size();
        cross[i].car_start_from_this=num;
        sort(tmp[i].begin(),tmp[i].end(),cmp_arange_time);
        double difference=1.0*MOD/num;
        for(int j=0;j<num;++j)
            car[tmp[i][j]].planTime=max(car[tmp[i][j]].planTime,int(round(difference*(j+1))));
       /* cout<<i<<" "<<num<<endl;
        int mod=min(3*num,MOD);
        int mi=0;
        for(int j=1;j<num;++j)
            if(car[tmp[i][j]].planTime<car[tmp[i][mi]].planTime) mi=j;
        for(int j=0;j<num;++j)
            if(mi!=j) car[tmp[i][j]].planTime+=rand()%mod;*/
    }
}
void solve(string path)
{
    //sort(car.begin(),car.end(),cmp2);
    dijkstra_init();
    for(int i=1;i<=T;++i) car[i].value=cal_value(car[i].from,car[i].to,car[i].speed,car[i].planTime);
    random_add_planTime(Lucky_number);

    auto it=car.begin();
    ++it;
    sort(it,car.end(),cmp_value);
    cout<<"sort ok!"<<endl;

    ofstream out(path);
    for(int i=1;i<=T;++i)
    {
        vector<int> ans;
        ans.clear();
        int len;

        /*int bounder=round(1.0*Lucky_number/cross[car[i].from].car_start_from_this);
        int l=max(car[i].initTime,car[i].planTime-bounder);
        int r=car[i].planTime+1;
        int mi=cross[car[l].from].query_car_num(l);
        for(int i=l+1;i<=r;++i)
            mi=min(mi,cross[car[i].from].query_car_num(i));
        vector<int> tmp;
        tmp.clear();
        for(int i=l;i<=r;++i)
            if(cross[car[i].from].query_car_num(i)==mi) tmp.push_back(i);
        len=rand()%(int(tmp.size()));
        car[i].planTime=tmp[len];*/
        //tmp.clear();
        //int start=Random_drive(car[i].from,0,tmp);
        //for(auto x:tmp) cout<<x<<" ";
        //cout<<endl;
        //int tmp_len=tmp.size();
      //  car[i].planTime+=rand()%100;
        ans=dijkstra(car[i].from,car[i].to,car[i].speed,car[i].planTime);
        //for(int i=tmp_len-1;i>=0;--i) ans.push_back(tmp[i]);
        ans.push_back(car[i].planTime);
        ans.push_back(car[i].id);
        //ans.push_back(car[i].from);
        reverse(ans.begin(),ans.end());
        out<<"(";
        len=ans.size();
        for(int i=0;i<len-1;++i) out<<ans[i]<<",";
        out<<ans[len-1]<<")"<<endl;
    }
}
void write(string path)
{

}

int main(int argc, char *argv[])
{
    srand(19260817);
    std::ios::sync_with_stdio(false);
    cin.tie(0);
    std::cout << "Begin" << std::endl;

    //////////////////////////////提交

	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
    std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);

	//////////////////////////////本地
    /*string carPath="..\\config\\car.txt";
    string roadPath="..\\config\\road.txt";
    string crossPath="..\\config\\cross.txt";
    string answerPath="..\\config\\answer.txt";*/


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;


	// TODO:read input filebuf

	readcar(carPath);
	readcross(crossPath);
	readroad(roadPath);

	// TODO:process
	solve(answerPath);
	/*freopen("ce.out","w",stdout);
	for(int i=1;i<=n;++i)
        for(int j=1;j<=500;++j)
        {
            cout<<"("<<i<<","<<j<<")"<<cross[i].query_car_num(j)<<endl;
        }*/
	// TODO:write output file
    //write(answerPath);
	return 0;
}

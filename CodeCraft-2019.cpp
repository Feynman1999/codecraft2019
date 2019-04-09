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
#include <set>
#include <unordered_map>
#define mp make_pair
using namespace std;
const double inf=1e17;
const int GO_STRAIGHT=0;
const int TURN_LEFT=1;
const int TURN_RIGHT=2;
const double LEFT_PENALTY=10;
const double RIGHT_PENALTY=30;
const int Lucky_number=1525;
const int Cross_penalty_alpha = 60;


unordered_map<int,int> Cross_id_to_index;
unordered_map<int,int> Car_id_to_index;
unordered_map<int,int> Road_id_to_index;
struct Road
{
    int id;
    int length;
    int speed;
    int channel;
    int from;
    int to;
    bool isDuplex;
    vector<double> mi;
    vector<int> num;
    void set(int l,int r,int v)
    {
        int last=mi.size();
        if(last<=r)
        {
            mi.resize(r+1);
            num.resize(r+1);
            for(int i=last;i<=r;++i) mi[i]=0,num[i]=0;
        }
        for(int i=l;i<=r;++i)
        {
            mi[i]=(mi[i]*num[i]+v)/(num[i]+1);
            num[i]+=1;
            //mi[i]=min(mi[i],v);
        }
    }
    double query(int l)
    {
        if(l>=mi.size()) return speed;
        if(num[l]==0) return speed;
        return min(1.0*speed,mi[l]);
    }
    void output()
    {
        cout<<id<<" "<<length<<" "<<speed<<" "<<channel<<" "<<from<<" "<<to<<" "<<isDuplex<<endl;
    }
};
struct Car
{
    int id;
    int from;
    int to;
    int speed;
    int initTime;
    bool priority;
    bool preset;
    int planTime;
    double value;
    vector<int> path;
    void output()
    {
        cout<<id<<" "<<from<<" "<<to<<" "<<speed<<" "<<planTime<<" "<<priority<<" "<<preset<<endl;
    }
};
bool cmp1(const Car &a,const Car &b)
{
    if(a.speed!=b.speed) return a.speed<b.speed;
    return a.planTime<b.planTime;
}
bool cmp2(const Car &a,const Car &b)
{
    //if(a.priority!=b.priority) return a.priority>b.priority;
    if(a.preset!=b.preset) return a.preset>b.preset;
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
        if(query_car_num(t)<=2*get_road_num()) return 0;
        double num=Cross_penalty_alpha*query_car_num(t)/get_road_num();
        //if(num<=get_road_num()*2) num=0;
        return num;

        /*if(query_car_num(t)<=get_road_num()*2) return 0;
        double num=query_car_num(t)*get_road_num();
        return num;*/
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
vector<Car> car;
int T;
vector<Road> road;
int m;
vector<Cross> cross;
int n;
void addedge(int from,int to,int id)
{
    //cout<<from<<" "<<to<<" "<<id;
    //cout<<from<<" "<<to<<" "<<id<<endl;
    g[from].push_back(mp(to,id));
}
void readcar(string path)
{
    ifstream in(path);
    string info;
    getline(in,info);
    car.clear();
    car.push_back(Car());
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
        s>>ch;
        s>>tmp.priority;
        s>>ch;
        s>>tmp.preset;
        tmp.planTime=tmp.initTime;
        Car_id_to_index[tmp.id]=T;
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
    road.push_back(Road());
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
        tmp.from=Cross_id_to_index[tmp.from];
        tmp.to=Cross_id_to_index[tmp.to];
        Road_id_to_index[tmp.id]=m;
        road.push_back(tmp);
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
    cross.push_back(Cross());
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
        Cross_id_to_index[tmp.id]=n;
        cross.push_back(tmp);
    }
    g.resize(n+1);
    for(int i=0;i<=n;++i) g[i].clear();
    //for(auto x:cross) x.output();
}
void readpresetAnswer(string path)
{
    ifstream in(path);
    string info;
    getline(in,info);
    while(getline(in,info))
    {
        char ch;
        stringstream s(info);
        int id;
        s>>ch;
        s>>id;
        id=Car_id_to_index[id];
        s>>ch;
        s>>car[id].planTime;
        while(true)
        {
            s>>ch;
            if(ch==')') break;
            int x;
            s>>x;
            car[id].path.push_back(x);
        }
    }
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
    d[S]=1.0*start_time;
    while(!q.empty()) q.pop();
    q.push(mp(1.0*start_time,S));
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
    for(int i=0;i<=n;++i) actual_d[i]=d[i]=inf,pre[i]=mp(0,0),done[i]=0;
    d[S]=1.0*start_time;
    //cross[S].set(int(d[S]));
    actual_d[S]=1.0*start_time;
    while(!q.empty()) q.pop();
    q.push(mp(1.0*start_time,S));
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
            double time_in_road=1.0*edge.length/min(edge.query(int(actual_d[u])),1.0*speed);
            double data=now.first+time_in_road+cross[v].query_penalty(int(actual_d[u]+time_in_road));
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
        //road[pre[now].second].set(int(actual_d[pre[now].first]),int(actual_d[now]),speed);
        //cross[now].set(int(actual_d[now]));
        now=pre[now].first;
    }
    //cross[now].set(int(actual_d[now]));
    reverse(ans.begin(),ans.end());
    return ans;
}
void update_model(double now,int u,const vector<int>& path ,int speed)
{
    cross[u].set(int(now));
    for(auto road_id:path)
    {
        Road &edge=road[Road_id_to_index[road_id]];
        //cout<<edge.id<<" ";
        double time_in_road=1.0*edge.length/min(edge.query(int(now)),1.0*speed);
        edge.set(int(now),int(now+time_in_road),speed);
        if(edge.from==u) u=edge.to;else u=edge.from;
        now+=time_in_road;
        cross[u].set(int(now));
        //cout<<u<<" "<<now<<endl;
    }
    //cout<<endl;
}
bool cmp_planTime(const int &x,const int &y)
{
    return car[x].planTime<car[y].planTime;
}
void random_add_planTime_priority(int lower,int upper)
{
    vector<vector<int>> tmp;
    tmp.resize(n+1);
    for(int i=1;i<=n;++i) tmp[i].clear();
    for(int i=1;i<=T;++i)
        if((!car[i].preset&&car[i].priority)||(car[i].preset&&car[i].planTime<=upper&&car[i].planTime>=lower))
           tmp[car[i].from].push_back(i);
    int sum=0;
    for(int i=1;i<=n;++i)
    {
        int num=tmp[i].size();
        sum+=num;
        //cout<<i<<" : "<<num<<endl;
        //cross[i].car_start_from_this=num;
        sort(tmp[i].begin(),tmp[i].end(),cmp_planTime);
        double difference=1.0*(upper-lower)/num;
        for(int j=0;j<num;++j)
            if(!car[tmp[i][j]].preset)
                car[tmp[i][j]].planTime=max(car[tmp[i][j]].planTime,lower+int(difference*j));
       // for(int j=0;j<num;++j) cout<<car[tmp[i][j]].planTime<<" ";
        //cout<<endl;
    }
    cout<<"priority cars (without preset) numbers : "<<sum<<endl;
}
void random_add_planTime(int lower,int upper)
{
    vector<vector<int>> tmp;
    tmp.resize(n+1);
    for(int i=1;i<=n;++i) tmp[i].clear();
    for(int i=1;i<=T;++i)
        if((!car[i].preset&&!car[i].priority)||(car[i].preset&&car[i].planTime<=upper&&car[i].planTime>=lower))
            tmp[car[i].from].push_back(i);
    for(int i=1;i<=n;++i)
    {
        int num=tmp[i].size();
       // cout<<i<<" : "<<num<<endl;
        //cross[i].car_start_from_this=num;
        sort(tmp[i].begin(),tmp[i].end(),cmp_planTime);
        double difference=1.0*(upper-lower)/num;
        for(int j=0;j<num;++j)
            if(!car[tmp[i][j]].preset)
                car[tmp[i][j]].planTime=max(car[tmp[i][j]].planTime,lower+int(difference*j));
    }
}
void solve(string path)
{
    //sort(car.begin(),car.end(),cmp2);
    dijkstra_init();
    random_add_planTime_priority(750,900);
    random_add_planTime(950,2400);
    for(int i=1;i<=T;++i) car[i].value=cal_value(car[i].from,car[i].to,car[i].speed,car[i].planTime);
    auto it=car.begin();
    ++it;
    sort(it,car.end(),cmp2);
    cout<<"sort ok!"<<endl;

    /*int early_planTime=100000,last_planTime=0;
    for(int i=1;i<=T;++i)
        if(car[i].priority)
            early_planTime=min(early_planTime,car[i].planTime),last_planTime=max(last_planTime,car[i].planTime);

    cout<<"early_planTime : "<<early_planTime<<"    last_planTime : "<<last_planTime<<endl;*/

    ofstream out(path);
    for(int i=1;i<=T;++i)
    {
        //car[i].output();
        vector<int> ans;
        ans.clear();
        if(car[i].preset) ans=car[i].path;
        else
            ans=dijkstra(car[i].from,car[i].to,car[i].speed,car[i].planTime);
        //cout<<car[i].id<<" : ";
        update_model(car[i].planTime,car[i].from,ans,car[i].speed);

      //  if(car[i].id==19051) car[i].output();

        if(!car[i].preset)
        {
            //for(int i=tmp_len-1;i>=0;--i) ans.push_back(tmp[i]);
            out<<"("<<car[i].id<<","<<car[i].planTime<<",";
            int len=ans.size();
            for(int i=0;i<len-1;++i) out<<ans[i]<<",";
            out<<ans[len-1]<<")"<<endl;
        }
    }


}
void cal_coefficient()
{
    int priority_car=0;
    for(int i=1;i<=T;++i)
        if(car[i].priority) ++priority_car;
    cout<<"0.05*"<<T<<"/"<<priority_car<<"="<<0.05*T/priority_car<<endl;

    int max_speed=0,min_speed=1000;
    int priority_max_speed=0,priority_min_speed=1000;
    for(int i=1;i<=T;++i)
    {
        max_speed=max(max_speed,car[i].speed);
        min_speed=min(min_speed,car[i].speed);
        if(car[i].priority)
        {
            priority_max_speed=max(priority_max_speed,car[i].speed);
            priority_min_speed=min(priority_min_speed,car[i].speed);
        }
    }
    cout<<"0.2375*("<<max_speed<<"/"<<min_speed<<")/("<<priority_max_speed<<"/"<<priority_min_speed<<")="<<0.2375*(1.0*max_speed/min_speed)/(1.0*priority_max_speed/priority_min_speed)<<endl;

    set<int> from,to,priority_from,priority_to;
    from.clear();
    to.clear();
    priority_from.clear();
    priority_to.clear();
    for(int i=1;i<=T;++i)
    {
        from.insert(car[i].from);
        to.insert(car[i].to);
        if(car[i].priority)
        {
            priority_from.insert(car[i].from);
            priority_to.insert(car[i].to);
        }
    }
    cout<<"0.2375*"<<from.size()<<"/"<<priority_from.size()<<"="<<0.2375*int(from.size())/int(priority_from.size())<<endl;
    cout<<"0.2375*"<<to.size()<<"/"<<priority_to.size()<<"="<<0.2375*int(to.size())/int(priority_to.size())<<endl;
}



int main(int argc, char *argv[])
{
    srand(19260817);
    std::ios::sync_with_stdio(false);
    cin.tie(0);
    std::cout << "Begin" << std::endl;
    /*
	if(argc < 6){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}

	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string presetAnswerPath(argv[4]);
	std::string answerPath(argv[5]);
	*/

    //////////////////////////////锟斤拷锟斤拷
    string carPath="..\\config\\car.txt";
    string roadPath="..\\config\\road.txt";
    string crossPath="..\\config\\cross.txt";
    string presetAnswerPath="..\\config\\presetAnswer.txt";
    string answerPath="..\\config\\answer.txt";


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;



	// TODO:read input filebuf

	readcar(carPath);
	readcross(crossPath);
	readroad(roadPath);
	for(int i=1;i<=T;++i) car[i].from=Cross_id_to_index[car[i].from];
	for(int i=1;i<=T;++i) car[i].to=Cross_id_to_index[car[i].to];
    readpresetAnswer(presetAnswerPath);
    //for(auto x:car[Car_id_to_index[21701]].path) cout<<x<<" ";cout<<endl;
	// TODO:process
	//cal_coefficient();

	solve(answerPath);
	/*freopen("ce.out","w",stdout);
	for(int i=1;i<=n;++i)
        for(int j=1;j<=500;++j)
        {
            cout<<"("<<i<<","<<j<<")"<<cross[i].query_car_num(j)<<endl;
        }*/
	// TODO:write output file
    //write(answerPath);
    //car[Car_id_to_index[19051]].output();
	return 0;
}
//(750~900 950~2400) 4089

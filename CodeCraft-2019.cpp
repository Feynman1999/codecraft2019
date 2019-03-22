#include "iostream"
#include "string"
#include "fstream"
#include "stdlib.h"
#include "vector"
#include "sstream"
#include "algorithm"
#include "queue"
#include "time.h"
#define mp make_pair
using namespace std;
const double inf=1e17;
const int GO_STRAIGHT=0;
const int TURN_LEFT=1;
const int TURN_RIGHT=2;
const double LEFT_PENALTY=1;
const double RIGHT_PENALTY=2;
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
struct Car
{
    int id;
    int from;
    int to;
    int speed;
    int planTime;
    void output()
    {
        cout<<id<<" "<<from<<" "<<to<<" "<<speed<<" "<<planTime<<endl;
    }
};
bool cmp1(const Car &a,const Car &b)
{
    if(a.speed!=b.speed) return a.speed<b.speed;
    return a.planTime<b.planTime;
}
bool cmp2(const Car &a,const Car &b)
{
    if(a.planTime!=b.planTime) return a.planTime<b.planTime;
    return a.speed<b.speed;
}
struct Cross
{
    int id;
    int dir[4];
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
    g[from].push_back(mp(to,id));
}
void readcar(string path)
{
    ifstream in(path);
    string info;
    getline(in,info);
    car.clear();
    car.push_back({0,0,0,0,0});
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
        s>>tmp.planTime;
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
vector<double> d;
vector<pair<int,int>> pre;   //<pre_cross,through_road>
void dijkstra_init()
{
    done.resize(n+1);
    d.resize(n+1);
    pre.resize(n+1);
}
priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> q;
int Random_drive(int S,int len,vector<int> &ans)
{
    for(int i=1;i<=len;++i)
    {
        int to=rand()%(int(g[S].size()));
        ans.push_back(road[g[S][to].second].id);
        S=g[S][to].first;
    }
    return S;
}
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
vector<int> dijkstra(int S,int T,int speed,int start_time)
{
    //cout<<S<<endl;
    for(int i=0;i<=n;++i) d[i]=inf,pre[i]=mp(0,0),done[i]=0;
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
            double data=now.first+1.0*edge.length/min(edge.query(int(now.first)),speed);
            int dir=checkdir(road[pre[u].second].id,u,edge.id);
            if(dir==TURN_LEFT) data+=LEFT_PENALTY;
            if(dir==TURN_RIGHT) data+=RIGHT_PENALTY;
            if(data<d[v])
            {
                pre[v]=mp(u,x.second);
                d[v]=data;
                q.push(mp(d[v],v));
            }
        }
    }
    vector<int> ans;
    cout<<d[T]<<endl;
    ans.clear();
    int now=T;
    while(now!=S)
    {
        ans.push_back(road[pre[now].second].id);
        road[pre[now].second].set(int(d[pre[now].first]),int(d[now]),speed);
        now=pre[now].first;
    }
    return ans;
}
bool cmp_planTime(const int &x,const int &y)
{
    return car[x].planTime<car[y].planTime;
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
        sort(tmp[i].begin(),tmp[i].end(),cmp_planTime);
        double difference=1.0*MOD/num;
        for(int j=0;j<num;++j)
            car[tmp[i][j]].planTime=max(car[tmp[i][j]].planTime,int(difference*j));
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
    random_add_planTime(600);
    auto it=car.begin();
    ++it;
    sort(it,car.end(),cmp2);
    cout<<"sort ok!"<<endl;
    dijkstra_init();
    ofstream out(path);
    for(int i=1;i<=T;++i)
    {
        vector<int> ans,tmp;
        ans.clear();
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
        reverse(ans.begin(),ans.end());
        out<<"(";
        int len=ans.size();
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
    string answerPath="..\\config\\answer.txt";


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;*/


	// TODO:read input filebuf

	readcar(carPath);
	readcross(crossPath);
	readroad(roadPath);

	// TODO:process
	solve(answerPath);
	// TODO:write output file
    //write(answerPath);
	return 0;
}

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
    bool operator < (const Car &x) const
    {
        if(speed!=x.speed) return speed<x.speed;
        return planTime<x.planTime;
    }
    void output()
    {
        cout<<id<<" "<<from<<" "<<to<<" "<<speed<<" "<<planTime<<endl;
    }
};
struct Cross
{
    int id;
    int up;
    int right;
    int down;
    int left;
    void output()
    {
        cout<<id<<" "<<up<<" "<<right<<" "<<down<<" "<<left<<endl;
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
        s>>tmp.up;
        s>>ch;
        s>>tmp.right;
        s>>ch;
        s>>tmp.down;
        s>>ch;
        s>>tmp.left;
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
vector<int> dijkstra(int S,int T,int speed,int start_time)
{
    //cout<<S<<endl;
    for(int i=0;i<=n;++i) d[i]=inf,pre[i]=mp(0,0),done[i]=0;
    d[S]=0;
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
            if(now.first+1.0*edge.length/min(edge.query(int(now.first)),speed)<d[v])
            {
                pre[v]=mp(u,x.second);
                d[v]=now.first+1.0*edge.length/min(edge.query(int(now.first)),speed);
                q.push(mp(d[v],v));
            }
        }
    }
    vector<int> ans;
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

void solve(string path)
{
    sort(car.begin(),car.end());

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
        car[i].planTime+=rand()%100;
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
   /* string carPath="..\\config\\car.txt";
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
	// TODO:write output file
    //write(answerPath);
	return 0;
}

#include<iostream>
#include<vector>
#include<fstream>
#include<string>
#include<sstream>
#include<cstdlib>
#include<deque>
#include<utility>

using namespace std;

vector<string> world;
const int world_size[2] = {12, 17};
const int start[2] = {world_size[0]-1, world_size[1]-1};
const int goal[2] = {0, 0};
const int motions[4][2] = {{-1, 0},
						   {0, -1},
						   {1, 0},
						   {0, 1}};

const int cost = 1;
void OpenFile(ifstream&);
void PopulateWorld(ifstream&);
void PerformValueIteration(vector<vector<int> >&, vector<vector<int> >&);
void OutputWorld(const vector<string>&);
void OutputValue(const vector<vector<int> >&);

int main()
{
	vector<vector<int> > value(world_size[0], vector<int>(world_size[1],99));
	vector<vector<int> > closed(world_size[0], vector<int>(world_size[1],0));
	ifstream input;
	
	OpenFile(input);
	
	PopulateWorld(input);
	
	PerformValueIteration(value, closed);
	OutputWorld(world);
	OutputValue(value);
	return 0;
}

void OpenFile(ifstream& input)
{
	string filename = "grid.txt";
	while (true)
	{
		input.open(filename.c_str());
		if(input.is_open())
		{
			break;
		}
		else
		{
			cerr<<"No such file \n";
			input.clear();
		}
	
	}

}

void PopulateWorld(ifstream& input)
{
	
	string line;
	
	while(getline(input,line))
	{	
		
		world.push_back(line);
	}

}

void OutputWorld(const vector<string>& world)
{
	cout << "World || 1 = obstacle, 0 = Free "<< endl;
	for(vector<string>::const_iterator i = world.begin(); i != world.end(); ++i)
	{
		cout<<*i<<endl;
	}
}
void OutputValue(const vector<vector<int> >& value)
{
	cout << "Value of each grid point: | 99 = obstacle" << endl;
	for(vector<vector<int> >::const_iterator i = value.begin(); i != value.end(); ++i)
	{
		for(vector<int>::const_iterator j = i->begin(); j != i->end(); ++j)
		{
			cout << *j << ", ";
		}
		cout<<endl;
	}
}

void PerformValueIteration(vector<vector<int> >& value, vector<vector<int> >& closed)
{
	deque<pair<int,int> > open; //Open list is a deque unlike priority queue in case of search
	bool finished = false, failed = false;
	int x = goal[0];
	int y = goal[1];
	value[x][y] = 0; //Value of each state is stored in a table
	closed[x][y] = 1;
	open.push_back(make_pair(x,y));
	int num_motions = (sizeof(motions)/sizeof(motions[0]));
	while(!finished && !failed)
	{
		if(open.size() == 0)
		{
			failed = true;
			cout << "Failed No Solution \n";
		}
		else
		{
			pair<int,int> curr = open.front();
			open.pop_front();
			x = curr.first;
			y = curr.second;
			if(x == start[0] && y == start[1])
			{
				finished = true;
			}
			else
			{
				for(int i =0; i < num_motions; ++i)
				{
					int x2 = x - motions[i][0];
					int y2 = y - motions[i][1];

					if (x2 >= 0 && x2 < world_size[0] && y2 >= 0 && y2 < world_size[1])
					{	
						
						if(int(world[x2][y2] - '0') == 0 && closed[x2][y2] == 0)
						{   
							value[x2][y2] = value[x][y] + cost;
							closed[x2][y2] = 1;
							open.push_back(make_pair(x2,y2));
						}
					}
				}
			}
		}
	}

}
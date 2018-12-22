#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>

#define SIZE            11               // n in nxn map size
#define RESOLUTION      0.2             // 20 cm
#define NODE_ORIGIN     (SIZE - 1) / 2

#define NODE_ID         1
#define NODE_PARENT     5
#define NODE_TYPE       7
#define NODE_F          2

#define NODE_TYPE_OBSTACLE          2
#define NODE_TYPE_DEST              3
#define NODE_TYPE_START             1
#define NODE_TYPE_FREE              0
#define NUM_OBSTACLES               2

typedef struct gate gate_t;
struct gate
{
    float left_post[2];
    float right_post[2];
};

typedef struct node node_t;

struct node 
{
    int id;
    int f;  // sum of g and h
    int g;  // cost to get to the square
    int h;  // hueristic cost to get to destination from square
    int parent;
    int type;
};

typedef struct point point_t;
struct point
{
    int x;
    int y;
};

typedef struct list list_t;
struct list
{
    point_t items[SIZE*SIZE];
    int     num_items;
};

typedef list_t path_t;

FILE* g_logfile;

node_t  g_map[SIZE][SIZE];
point_t g_obstacles[NUM_OBSTACLES];
list_t  g_openList;
int     g_closed[SIZE][SIZE];

void initializeMap();
void initializeLists(point_t* start);
void addPointToList(point_t*);
void displayMap(int field);
void displayPoint(char* name, point_t* p);
int  isValidIndex(int row, int col);
void createPoint(point_t* p, double x, double y);
void setCellOnMap(point_t* p, int f, int g, int h, int parent, int type);
void removePointFromList(point_t* p, int idx);
int  getID(point_t* p);
int  findSmallestF(point_t* smallestF);
void displayList(list_t* l);
int  checkCell(int x, int y, point_t* p, point_t* dest, int* destFound);
int  calcHueristic(int x, int y, point_t* dest);
void expandObstacles();
void createPath(path_t* route, point_t* goal, point_t* start);
void createGoal(gate_t* gate, point_t* goal);
int aStarSearch(path_t*, point_t* start, gate_t* gate);

int main()
{
    point_t start, goal, left_post, right_post;
    path_t route;
    double start_x = -0.8, start_y = -0.6;
    gate_t gate1;
    gate1.left_post[0] = -0.3;
    gate1.left_post[1] = 0.3;
    gate1.right_post[0] = 0.3;
    gate1.right_post[1] = 0.3;

    g_logfile = fopen("log.txt", "w");
    if (g_logfile == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    createPoint(&start, start_x, start_y);
    if (aStarSearch(&route, &start, &gate1) == 0)
    {
        printf("Failed to find the goal\n"); 
    }

    displayMap(NODE_TYPE);
    //displayPoint("Start", &start);
    //displayList(&g_openList);
    fclose(g_logfile);
    return 0;
}

int aStarSearch(path_t* route, point_t* start, gate_t* gate)
{
    point_t goal, left_post, right_post;
    createPoint(&left_post, gate->left_post[0], gate->left_post[1]);
    createPoint(&right_post, gate->right_post[0], gate->right_post[1]);
    initializeMap();
    
    setCellOnMap(start, 0, 0, 0, getID(start), NODE_TYPE_START);
    setCellOnMap(&left_post, INT_MAX, INT_MAX, INT_MAX, -1, NODE_TYPE_OBSTACLE);
    setCellOnMap(&right_post, INT_MAX, INT_MAX, INT_MAX, -1, NODE_TYPE_OBSTACLE);
    createGoal(gate, &goal);
    g_obstacles[0] = left_post;
    g_obstacles[1] = right_post;
    expandObstacles();

    initializeLists(start);
    int destFound = 0;
    while (g_openList.num_items > 0)
    {
        //displayList(&g_openList);
        point_t q;
        int q_id = findSmallestF(&q);
        //displayPoint("Exploring Point", &q);
        //printf("ID of smallest f: %d\n", q_id);
        if (q_id == -1)
        {
            printf("Error, no elements in open list\n");
        }

        removePointFromList(&q, q_id);
        g_closed[q.x][q.y] = 1;  // Add point to close list

        if (checkCell(q.x-1,q.y, &q, &goal, &destFound)) break;
        if (checkCell(q.x,q.y+1, &q, &goal, &destFound)) break;
        if (checkCell(q.x+1,q.y, &q, &goal, &destFound)) break;
        if (checkCell(q.x,q.y-1, &q, &goal, &destFound)) break;

        //displayMap(NODE_F);
        //printf("Have not found dest yet!\n");
    }
    createPath(route, &goal, start);
    return destFound;
}

void createPath(path_t* route, point_t* goal, point_t* start)
{
    for (int i = 0; i < SIZE*SIZE; i++)
    {
        route->items[i].x = -1;
        route->items[i].y = -1;
    }

    // Add destination to route
    route->items[0].x = goal->x;
    route->items[0].y = goal->y;
    route->num_items++;

    // Follow parent
    int x = goal->x, y = goal->y, id = g_map[x][y].parent;
    while(id != getID(start))
    {
        route->items[route->num_items].x = id/SIZE;
        route->items[route->num_items].y = id%SIZE;
        route->num_items++;
        id = g_map[id/SIZE][id%SIZE].parent;
    }

    // Add start to route
    route->items[route->num_items].x = start->x;
    route->items[route->num_items].y = start->y;
    route->num_items++;

    displayList(route);
}

void createGoal(gate_t* gate, point_t* goal)
{
    double y, x = (gate->left_post[0] + gate->right_post[0]) / 2;
    if (gate->left_post[0] > gate->right_post[0])
    {
        y = (gate->left_post[1] + gate->right_post[1]) / 2 + 2 * RESOLUTION;
    }
    else 
    {
        y = (gate->left_post[1] + gate->right_post[1]) / 2 - 2 * RESOLUTION;
    }
    createPoint(goal, x, y);
    setCellOnMap(goal, INT_MAX, INT_MAX, INT_MAX, -1, NODE_TYPE_DEST);
}

int calcHueristic(int x, int y, point_t* goal)
{
    // Manhattan distance
    return abs(x - goal->x) + abs(y - goal->y);
}

int checkCell(int x, int y, point_t* p, point_t* goal, int* destFound)
{
    if (isValidIndex(x,y))
    {

        if (g_map[x][y].type == NODE_TYPE_DEST)
        {
            g_map[x][y].parent = getID(p);
            fprintf(g_logfile, "Found the destination at %d, %d\n", x,y);
            *destFound = 1;
            return 1;
        }
        else if (g_closed[x][y] == 0 && g_map[x][y].type != NODE_TYPE_OBSTACLE)
        {
            int gNew = g_map[p->x][p->y].g + 1;
            int hNew = calcHueristic(x,y,goal);
            int fNew = gNew + hNew;

            if (g_map[x][y].f == INT_MAX || g_map[x][y].f > fNew)
            {
                point_t node;
                node.x = x; node.y = y;
                
                // Only add node if it is not on list already
                if (g_map[x][y].f == INT_MAX )
                {
                    
                    addPointToList(&node);
                }
                
                setCellOnMap(&node, fNew, gNew, hNew, getID(p), NODE_TYPE_FREE);
            }
        }
    }
    return 0;
}

int isValidIndex(int row, int col)
{
    return (row >= 0) && (row < SIZE) && (col >= 0) && (col < SIZE); 
}

int getID(point_t* p)
{
    return p->x*SIZE + p->y;
}

void setCellOnMap(point_t* p, int f, int g, int h, int parent, int type)
{
    g_map[p->x][p->y].f = f;
    g_map[p->x][p->y].g = g;
    g_map[p->x][p->y].h = h;
    g_map[p->x][p->y].parent = parent;
    g_map[p->x][p->y].type = type;
}

int findSmallestF(point_t* p)
{
    int smallest = -1, x, y, numValids = 0, f = INT_MAX;
    for (int i = 0; i < SIZE*SIZE; i++)
    {
        x = g_openList.items[i].x;
        y = g_openList.items[i].y;
        if (isValidIndex(x, y) && g_map[x][y].f < f)
        {
            smallest = i;
            p->x = x;
            p->y = y;
            numValids++;
            f = g_map[x][y].f;
        }
        
        if (numValids == g_openList.num_items)
        {
            break;
        }
    }
    return smallest;
}

void initializeLists(point_t* start)
{
    for (int i = 0; i < SIZE*SIZE; i++)
    {
        g_openList.items[i].x = -1;
        g_openList.items[i].y = -1;
        g_closed[i/SIZE][i%SIZE] = 0;
    }
    addPointToList(start);
}

void addPointToList(point_t* p)
{
    int i;
    for (i = 0; i < SIZE*SIZE; i++)
    {
        if (g_openList.items[i].x == -1 && g_openList.items[i].y == -1)
        {
            g_openList.items[i].x = p->x;
            g_openList.items[i].y = p->y;
            g_openList.num_items++;
            break;
        }
    }
    if (i == SIZE*SIZE)
    {
        fprintf(g_logfile, "No more space in list\n");
    }
}

void removePointFromList(point_t* p, int idx)
{
    if (isValidIndex(idx/SIZE, idx%SIZE))
    {
        g_openList.items[idx].x = -1;
        g_openList.items[idx].y = -1;
        g_openList.num_items--;
    }
    else
    {
        printf("Error, trying to remove a point outside of size of list\n");
    }
}

void expandObstacles()
{
    for (int i = 0; i < NUM_OBSTACLES; i++)
    {
        int x = g_obstacles[i].x - 1;
        int y = g_obstacles[i].y - 1;

        if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        x++; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        x++; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        y++; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        y++; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        x--; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        x--; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
        y--; if (isValidIndex(x,y) && g_map[x][y].type == NODE_TYPE_FREE) g_map[x][y].type = NODE_TYPE_OBSTACLE;
    }
}

void createPoint(point_t* p, double x, double y)
{
    p->x = (int)(round(-x / RESOLUTION)) + NODE_ORIGIN;
    p->y = (int)(round(-y / RESOLUTION)) + NODE_ORIGIN;
}

void initializeMap()
{
    for (int i = 0; i < SIZE; ++i)
    {
        for (int j = 0; j < SIZE; ++j)
        {
            g_map[i][j].id = i*SIZE + j;
            g_map[i][j].f = INT_MAX;
            g_map[i][j].g = INT_MAX;
            g_map[i][j].h = INT_MAX;
            g_map[i][j].parent = -1;
            g_map[i][j].type = NODE_TYPE_FREE;
        }
    }
}

void displayMap(int field)
{
    if(field == NODE_ID)
    {
        for (int i = 0; i < SIZE; ++i)
        {
            for (int j = 0; j < SIZE; ++j)
            {
                fprintf(g_logfile, "%d\t", g_map[i][j].id);
            }
            fprintf(g_logfile, "\n");
        }
    }
    else if(field == NODE_PARENT)
    {
        for (int i = 0; i < SIZE; ++i)
        {
            for (int j = 0; j < SIZE; ++j)
            {
                fprintf(g_logfile, "%d\t", g_map[i][j].parent);
            }
            fprintf(g_logfile, "\n");
        }
    }
    else if(field == NODE_TYPE)
    {
        for (int i = 0; i < SIZE; ++i)
        {
            for (int j = 0; j < SIZE; ++j)
            {
                fprintf(g_logfile, "%d\t", g_map[i][j].type);
            }
            fprintf(g_logfile, "\n");
        }
    }
    else if(field == NODE_F)
    {
        fprintf(g_logfile, "Displayin Map...");
        for (int i = 0; i < SIZE; ++i)
        {
            for (int j = 0; j < SIZE; ++j)
            {
                fprintf(g_logfile, "%d\t", g_map[i][j].f);
            }
            fprintf(g_logfile, "\n");
        }
        fprintf(g_logfile, "\n\n");
    }
}

void displayPoint(char* name, point_t* p)
{
    fprintf(g_logfile, "%s | X: %d, Y: %d\n", name, p->x, p->y);
}

void displayList(list_t* l)
{
    fprintf(g_logfile, "Open List X  Y\n");
    fprintf(g_logfile, "Num Elem:\t%d\n", l->num_items);
    fprintf(g_logfile, "---------\n");
    for (int i = 0; i < SIZE*SIZE; i++)
    {
        fprintf(g_logfile, "Entry %d:  %d, %d\n", i, l->items[i].x, l->items[i].y);
    }
}
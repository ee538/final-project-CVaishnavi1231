#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    double lat = 0.0;
    for (auto j = data.begin(); j != data.end(); j++){
      std::string id1 = data[j->first].id;
      if(id1 == id ){
        lat = data[j->first].lat;
    }
  }
    return lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    double lon = 0.0;
    for (auto j = data.begin(); j != data.end(); j++){
    std::string id1 = data[j->first].id;
    if(id1 == id ){
      lon = data[j->first].lon;
    }
  }
    return lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    std::string name = "";
    for (auto j = data.begin(); j != data.end(); j++){
      std::string id1 = data[j->first].id;
      if(id1 == id ){
        name = data[j->first].name;
    }
  }
    return name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    std::vector<std::string> n_id;
    for (auto j = data.begin(); j != data.end(); j++){
    std::string id1 = data[j->first].id;
    if(id1 == id ){
      n_id = data[j->first].neighbors;
    }
  }
    return n_id;
}


/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string id1 = "";
  for (auto j = data.begin(); j != data.end(); j++){
      std::string name1 = data[j->first].name;
      if(name1 == name ){
        id1 = data[j->first].id;
    }
  }
    return id1;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> pos(-1, -1);
  std::string id = GetID(name) ;
    if(id == "" ){
      return pos;
    } 
    pos.first = GetLat(id);
    pos.second = GetLon(id);
    
  return pos;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){

    int len1 = a.size();
    int len2 = b.size();
        
    int dp[len1+1][len2+1];

    for(auto i=0;i<=len1;i++)
    {
      dp[i][0]=i;
    }
        
    for(auto i=1;i<=len2;i++)
    {
      dp[0][i]=i;
    }
        
    for(auto i=1;i<=len1;i++)
    {
      for(int j=1;j<=len2;j++)
      {
        if(a[i-1]==b[j-1])
          dp[i][j]=dp[i-1][j-1];
        else{
          dp[i][j]=std::min(dp[i-1][j],dp[i][j-1]);
          dp[i][j]=1+std::min(dp[i][j],dp[i-1][j-1]);
        }
      }
    }
    return dp[len1][len2];
  }

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string temp = "";
  int res = INT_MAX;
  for(auto j = data.begin(); j != data.end(); j++){
    std::string name1 = data[j->first].name;
    int dist = CalculateEditDistance(name1, name);
    if (dist<res){
      res = dist;
      temp = name1;
    }

  }
  return temp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  transform(name.begin(), name.end(), name.begin(), ::tolower);
  for(auto j = data.begin(); j != data.end(); j++){
    std::string str_orig = data[j->first].name;
    std::string temp_str = data[j->first].name;
    transform(temp_str.begin(), temp_str.end(), temp_str.begin(), ::tolower);
    if(temp_str.find(name) == 0){
      results.push_back(str_orig);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::string start = GetID(location1_name);
  std::string end = GetID(location2_name); 
  if(start == "" || end == ""){
    return path;
  }
  std::priority_queue <std::pair<double,std::string>,std::vector<std::pair<double,std::string>>,std::greater<std::pair<double,std::string>>> pq;                 //Priority queue to implement Heap with <distance,node ID>
  std::unordered_map <std::string,double> dist;                 
  std::unordered_map <std::string,std::string> pre;   
  std::unordered_map <std::string,bool> visited;        
    
  for(auto j = data.begin(); j != data.end(); j++){
      dist[data[j->first].id] = DBL_MAX;
      pre[data[j->first].id] = "";
      visited[data[j->first].id] = false;
  }
    
  dist[start] = 0;
  pq.push(std::make_pair(dist[start],start));
  
  while(!pq.empty()){
    std::pair<double,std::string> pair_pq = pq.top();
    std::string current_id = pair_pq.second;
    pq.pop();
    if(current_id!=end){
      if(CalculateDistance(current_id,start)>dist[current_id]){
        continue;
      }
      else if(visited[current_id]){
        continue;
      }
      else{
        visited[current_id] = true;
        std::vector <std::string> neighbor_id_list  = data[current_id].neighbors;
        for(auto n : neighbor_id_list){
            double new_dist = dist[current_id] + CalculateDistance(current_id,n);
             if(new_dist < dist[n]){
               dist[n] = new_dist;
               pre[n] = current_id;   
               pq.push(std::make_pair(dist[n],n));          
              }
        }
      }
    }
    else{
      visited[end] = true;
      break;
    }
  }

  if(!visited[end]){
    return path;   
  }
     
  for(auto node = end; node!= start; node = pre[node])
  {
    path.push_back(node);
  }

  path.push_back(start);
  std::reverse(path.begin(),path.end());
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  std::vector<std::string> path;
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    location_names_from_csv.push_back(line);
  }  
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::string temp;
    std::vector<std::string> dependencies;
    while(getline(s, temp, ',')) {
      dependencies.push_back(temp);
    }
  dependencies_from_csv.push_back(dependencies);
  }
  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */

void TrojanMap::topologicalsort_helper(std::string loc,std::map<std::string, bool> &visited,std::stack<std::string> &loc_stack,std::unordered_map<std::string, std::vector<std::string>> adj ){
  visited[loc] = true;
  std::vector<std::string> adj_list = adj[loc];
  for(auto i:adj_list){
    if(!visited[i]){
      topologicalsort_helper(i,visited,loc_stack,adj);
    }
  }
  loc_stack.push(loc);
}

std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;

  //creating DAG

  std::unordered_map<std::string, std::vector<std::string>> adj;

  for(auto l:locations){
    std::vector<std::string> temp;
    adj[l] = temp;
  }

  for(auto d:dependencies){
    if(adj.find(d[1]) != adj.end()){
      std::vector<std::string> temp_list = adj[d[1]];
      if(std::find(temp_list.begin(), temp_list.end(), d[0]) != temp_list.end()){
        std::vector<std::string> temp_res;
        return temp_res;
      }
    }
    adj[d[0]].push_back(d[1]);
  }
  std::map<std::string, bool> visited;

  std::stack<std::string> loc_stack;

  for(auto l:locations){
    visited[l] = false;
  }
  for (int i = 0; i < locations.size(); i++)
    if (visited[locations[i]] == false)
      topologicalsort_helper(locations[i], visited, loc_stack, adj);
 
    while(!loc_stack.empty()) {
        result.push_back(loc_stack.top()); 
        loc_stack.pop();
    }
  return result;                                                          
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

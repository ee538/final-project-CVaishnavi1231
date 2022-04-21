## EE538 Final Project - Spring 2022 - TrojanMap

## TEAMMATES:
## 1. Vaishnavi Channakeshava
## 2. Sanjana Vasudeva

## 1. High-level overview of your design (Use diagrams and pictures for your data structures).


## 2. Detailed description of each function and its time complexity. 

 
1. GetLat

The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the id of the node(location) of that correspoding key. We compare the id given as the input to the function against the id we have extracted for each iteration. If the ID's are equal then we extract the third element of the Node which is the latitude.

2. GetLon

The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the id of the node(location) of that correspoding key. We compare the id given as the input to the function against the id we have extracted for each iteration. If the ID's are equal then we extract the third element of the Node which is the longitude.

3. GetName

The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the id of the node(location) of that correspoding key. We compare the id given as the input to the function against the id we have extracted for each iteration. If the ID's are equal then we extract the fourth element of the Node which is the name.


4. GetID

The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the name of the node(location) of that correspoding key. We compare the name given as the input to the function against the name we have extracted for each iteration. If the name's are equal then we extract the first element of the Node which is the ID.

5. GetNeighborIDs

The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the id of the node(location) of that correspoding key. We compare the id given as the input to the function against the id we have extracted for each iteration. If the ID's are equal then we extract the third element of the Node which is neighbors.


6. Autocomplete

The input to the function Autocomplete name which is a partial name is converted to lower case to avoid any discrepancy. The for loop is used to traverse over the data where in we are dereferencing the iterator to get the first member (Key) and we are extracting the name of the node(location) of that correspoding key and creating a copy of the name extracted. The copy of the name is also converted to lower case and this is compared against the partial name converted to lower case, if the partial string is at found at the zeroth index of the name (i.e the partial name is the prefix), then the location name is pushed into a vector. The vector containing the list of all names which has the partial name as prefix is returned. 


7. GetPosition

First the id corresponding to the given name is extracted using the GetID function. If the Id obtained is an empty string then a default pair is returned with latitude and longitude set to -1 else we get the latitude and logitude using the GetLat and GetLon function for the corresponding id and assign it to the first and second element of the pair and return the pair
 

8. CalculateEditDistance

A 2D array of size - (m+1)x(n+1) is created, where m = length of string1 and n = length of string2. A element of the 2D array at the ith, jth position D[i, j] = edit distance between length-i prefix of x and length-j prefix of y. A value in a cell depends upon its upper, left, and upper-left neighbors.
We have initialized D[0, j] to j, D[i, 0] to i. Then we fill the remaining cells from top row to bottom and from left to right.Then start from the end and choose the path which has a predecessor of minimum value.

The time complexity of the function is O(mn).


9. CalculateShortestPath_Dijkstra

The main goal of Dijkstra's algorithm is to find the shortest path between nodes in a graph. Firstly, we get the start and end node from the given location names. We create a min heap using priority queue with the pair <distance,node ID>. Next we use three unordered maps to store the shortest distance of each node, to save the predecessor, to save the visited nodes. We initialize all the id values of the shortest distance map to, all the id values of the predecessor map to empty string and set visited to false. The start node is then added to the min-heap. We use the while loop to find the shortest path using the predecessor map by removing the node from the min-heap and update the map until the destination node is found or until the heap is empty.

Time taken by function: 107 ms

Time Complexity : O((M + N)log(N))

<p align="center"><img src="img/ralphs_target_dijkstra.png" alt="Trojan" width="500" /></p>
<p align = "center">
Dijkstra path from Ralphs to Target
</p>

<p align="center"><img src="img/cafe_fashion.png" alt="Trojan" width="500" /></p>
<p align = "center">
Dijkstra path from 23rd Street Cafe to Fashion District
</p>


11. ReadLocationsFromCSVFile

The function ReadLocationsFromCSVFile takes CSV filename as the input, reads it and parse the locations data from CSV file. The locations name is appended to the vector which in turn is used for the topological sort problem.

Time Complexity : O(N)

12. ReadDependenciesFromCSVFile

The function ReadDependenciesFromCSVFile takes CSV filename whic has the source and ddestination names as the input, reads it and parse the dependencise data from CSV file. We create a vector of two strings i.e the source and destination. This vector pair is appendend to another vector which in turn is used for the topological sort problem.

Time Complexity : O(M*N)

16. Topologicalsort_helper

The Topologicalsort_helper is a recursive function. The visited map is updated to true for the given location. Next, the list of adjacent locations is obtained from the directed a-cyclic graph. We iterate over this list and if the location has not been visited we call the helper function recursively on this function. Next, we update the stack with the location given to the helper function.

Time Complexity : O(N)

17. DeliveringTrojan

This function creates directed a-cyclic graph using dependecies vector. To keep track of the id visited we use a visited map with string a boolean as the pair wherein the boolean value is set to false initially for all the points in the location vector. We also create a stack to store the locations that are visited. The helper function is invoked if the location is not visited. Next, the contets of the stack is pushed to the resulting vector.

Time taken by function: 0 ms

Time Complexity : O(M*N)

<p align="center"><img src="img/topological_1.png" alt="Trojan" width="500" /></p>
<p align = "center">
Topological sorted path of Ralphs to Chick-Fil-A to KFC
</p>

<p align="center"><img src="img/topological_2.png" alt="Trojan" width="500" /></p>
<p align = "center">
Topological sorted path of Target to 901 Bar & Grill to The Caribbean Apartments to Parkside Dining Hall to Social Security Administration
</p>

<p align="center"><img src="img/topological_3.png" alt="Trojan" width="500" /></p>
<p align = "center">
Topological sorted path of Food 4 Less to Chevron 2 to Adams-Normandie to Ralphs
</p>

## 3. Time spent for each function.

In this section we compute the time spent for each function.

1. Autocomplete : The time taken by the function to execute and give the result is approximately 22-29ms.

2. Find the location : The time taken by the function to execute and give the result is approximately 16ms.




## 4. Discussion, conclusion, and lessons learned.
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


## 3. Time spent for each function.

In this section we compute the time spent for each function.

1. Autocomplete : The time taken by the function to execute and give the result is approximately 22-29ms.

2. Find the location : The time taken by the function to execute and give the result is approximately 16ms.




## 4. Discussion, conclusion, and lessons learned.
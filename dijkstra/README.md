# dijkstra

## Overview
This is a basic graph based dijkstra search implementation in C++17.  
Dijkstra algorithm is just a special case of A-star algorithm with heuristic weight equal to 0.  
  
Run the [a-star](../a-star) algorithm with the `-w 0` parameter.  
Alternatively, you could use the [*zero_heuristic*](../a-star/src/search.h) functor as input to the 
[search_x](../a-star/src/search.h) functions (example use in [a_star.cpp](../a-star/src/a_star.cpp)).

Please see the [a-star](../a-star) project for more details on its use.


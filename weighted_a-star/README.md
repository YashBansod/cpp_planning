# weighted_a-star

## Overview
This is a basic graph based weighted a-star search implementation in C++17.  
Weighted A-star algorithm is just a special case of A-star algorithm with heuristic weight equal to `k`, where k is an
arbitrary integer value (usually > 1).  
  
Run the [a-star](../a-star) algorithm with the `-w k` parameter.  
Alternatively, you could define your own heuristic functor (see some examples in [search.h](../a-star/src/search.h))
and use it as the heuristic input to the [search_x](../a-star/src/search.h) functions 
(example use in [a_star.cpp](../a-star/src/a_star.cpp)).

Please see the [a-star](../a-star) project for more details on its use.


#include <vector>
#include <iterator>
#include <algorithm>
#include <utility>
#include <numeric>

// Reorder a vector values based off of a vector of indexes
// This implementation destroys the vector of indexes, but is extremely fast
template< typename order_iterator, typename value_iterator >
void reorder_destructive( order_iterator order_begin, order_iterator order_end, value_iterator v )  
{
  typedef typename std::iterator_traits< value_iterator >::value_type value_t;
  typedef typename std::iterator_traits< order_iterator >::value_type index_t;
  typedef typename std::iterator_traits< order_iterator >::difference_type diff_t;

  diff_t remaining = order_end - 1 - order_begin;
  for ( index_t s = index_t(); remaining > 0; ++ s ) 
  {
    index_t d = order_begin[s];
    if ( d == (diff_t) -1 ) continue;
    --remaining;
    value_t temp = v[s];
    for ( index_t d2; d != s; d = d2 ) 
    {
      std::swap( temp, v[d] );
      std::swap( order_begin[d], d2 = (diff_t) -1 );
      --remaining;
    }
    v[s] = temp;
  }
}

// Reorder a vector values based off of a vector of indexes
// This implementation does not destroy the vector of indexes,
// but it is 16% slower than reorder_destructive()
template< typename order_iterator, typename value_iterator >
void reorder( order_iterator order_begin, order_iterator order_end, value_iterator v )  
{   
    typedef typename std::iterator_traits< value_iterator >::value_type value_t;
    typedef typename std::iterator_traits< order_iterator >::value_type index_t;
    typedef typename std::iterator_traits< order_iterator >::difference_type diff_t;

    diff_t remaining = order_end - 1 - order_begin;
    for ( index_t s = index_t(), d; remaining > 0; ++ s ) 
		{
        for ( d = order_begin[s]; d > s; d = order_begin[d] ) ;
        if ( d == s ) 
        {
            -- remaining;
            value_t temp = v[s];
            while ( d = order_begin[d], d != s ) 
            {
              std::swap( temp, v[d] );
                -- remaining;
            }
            v[s] = temp;
        }
    }
}

// Returns a vector of the original indexes of the sorted elements
// based off of the compare function
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v, bool (*comp)(const T a, const T b))
{
  std::vector<size_t> idx(v.size());
  // Initialize the index vector, 0th indexed
  iota(idx.begin(), idx.end(), 0);

  // Sort based off of binary function
  std::sort(idx.begin(), idx.end(),
      [&](size_t i1, size_t i2) { return comp(v[i1], v[i2]); });

  return idx;
}

static bool comp_x(cv::Point2f& a, cv::Point2f& b) { return (a.x > b.x); }
static bool comp_y(cv::Point2f& a, cv::Point2f& b) { return (a.y > b.y); }
static bool comp_double(double a, double b) { return a < b; }


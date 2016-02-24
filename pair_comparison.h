#ifndef PAIR_COMPARISON_H_INCLUDED
#define PAIR_COMPARISON_H_INCLUDED


// Taken from here: http://www.cplusplus.com/reference/set/set/set/
// Sorts the elements in DESCENDING order
struct pairComparison
{
    bool operator() (std::pair<double, int> lhs, std::pair<double, int> rhs) const
        { return lhs.first > rhs.first; }
};


#endif // PAIR_COMPARISON_H_INCLUDED

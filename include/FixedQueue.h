#include <queue>
#include <deque>
#include <vector>
#include <map>
#include <iostream>

namespace ByteTrack{
template <typename T, int MaxLen, typename Container=std::deque<T>>
class FixedQueue : public std::queue<T, Container> {
public:
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }

    int most_frequent_element(){
        std::map<int, int> frequencyMap;
        int maxFrequency = 0;
        int mostFrequentElement = 0;
        //for(int i = 0; i < this->size(); i++)
        for(auto& x:this->c)
        {
            int f = ++frequencyMap[x];
            if (f > maxFrequency)
            {
                maxFrequency = f;
                mostFrequentElement = x;
            }
        }

        return mostFrequentElement;
    }
};
} // namespace ByteTrack
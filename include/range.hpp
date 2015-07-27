#ifndef PCL_COMPRESS_RANGE_H_
#define PCL_COMPRESS_RANGE_H_

#include <utility>

namespace pcl_compress {

template <class Iter>
class range : public std::pair<Iter, Iter> {
   public:
    range(const std::pair<Iter, Iter>& x) : std::pair<Iter, Iter>(x) {}
    virtual ~range() {}

    Iter
    begin() const {
        return this->first;
    }
    Iter
    end() const {
        return this->second;
    }
};

}  // pcl_compress

#endif /* PCL_COMPRESS_RANGE_H_ */

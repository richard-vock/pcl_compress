#ifndef PCL_COMPRESS_QUADTREE_HPP_
#define PCL_COMPRESS_QUADTREE_HPP_

#include <memory>
#include <vector>
#include <deque>

#include "types.hpp"
#include "range.hpp"

namespace pcl_compress {

class quadtree {
   public:
    typedef std::shared_ptr<quadtree> ptr_t;
    typedef std::shared_ptr<const quadtree> cptr_t;

    typedef struct params_ {
        uint32_t max_depth;
        uint32_t max_points_per_cell;
    } params_t;

    class node;
    class node_iterator;
    class leaf_iterator;
    typedef std::shared_ptr<node> node_ptr_t;
    typedef std::shared_ptr<const node> node_cptr_t;

   public:
    quadtree(const std::vector<vec2f_t>& points, const params_t& params);
    virtual ~quadtree();

    node_iterator nodes_begin();
    node_iterator nodes_end();

    leaf_iterator leaves_begin();
    leaf_iterator leaves_end();

    range<node_iterator> nodes();
    range<leaf_iterator> leaves();

   protected:
    node_ptr_t root_;
};

class quadtree::node {
   public:
    typedef std::shared_ptr<node> ptr_t;
    typedef std::weak_ptr<node> wptr_t;
    typedef std::shared_ptr<const node> const_cptr_t;
    typedef std::weak_ptr<const node> const_cwptr_t;
    typedef std::vector<int> indices_t;

   public:
    node(const bbox2f_t& bbox, const std::vector<vec2f_t>& points,
         const indices_t& subset, const params_t& params, uint32_t depth = 0);
    virtual ~node();

    indices_t& indices();
    const indices_t& indices() const;

    std::vector<ptr_t>& children();
    const std::vector<ptr_t>& children() const;

   protected:
    static bool inside_(uint32_t quadrant, const vec2f_t& point,
                        const vec2f_t& center);

   protected:
    std::vector<ptr_t> children_;
    bbox2f_t bbox_;
    indices_t indices_;
};

class quadtree::node_iterator {
   public:
    node_iterator();
    node_iterator(node::wptr_t node);
    virtual ~node_iterator();

    virtual node_iterator& operator++();

    bool operator==(const node_iterator& other);
    bool operator!=(const node_iterator& other);
    explicit operator bool() const;

    node& operator*();
    const node& operator*() const;

   protected:
    void update_queue_();

   protected:
    node::wptr_t node_;
    std::deque<node::wptr_t> queue_;
};

class quadtree::leaf_iterator : public quadtree::node_iterator {
   public:
    leaf_iterator();
    leaf_iterator(node::wptr_t node);
    virtual ~leaf_iterator();

    leaf_iterator& operator++();

   protected:
    node::wptr_t node_;
    std::deque<node::wptr_t> queue_;
};

}  // pcl_compress

#endif /* PCL_COMPRESS_QUADTREE_HPP_ */

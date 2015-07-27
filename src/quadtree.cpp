#include <quadtree.hpp>

namespace pcl_compress {

quadtree::quadtree(const std::vector<vec2f_t>& points, const params_t& params) {
    bbox2f_t bbox;
    for (const auto& p : points) {
        bbox.extend(p);
    }
    node::indices_t subset(points.size());
    std::iota(subset.begin(), subset.end(), 0);
    root_ = std::make_shared<node>(bbox, points, subset, params);
}

quadtree::~quadtree() {}

quadtree::node_iterator
quadtree::nodes_begin() {
    return node_iterator(node::wptr_t(root_));
}

quadtree::node_iterator
quadtree::nodes_end() {
    return node_iterator();
}

range<quadtree::node_iterator>
quadtree::nodes() {
    return range<node_iterator>(std::make_pair(nodes_begin(), nodes_end()));
}

quadtree::leaf_iterator
quadtree::leaves_begin() {
    return leaf_iterator(node::wptr_t(root_));
}

quadtree::leaf_iterator
quadtree::leaves_end() {
    return leaf_iterator();
}

range<quadtree::leaf_iterator>
quadtree::leaves() {
    return range<leaf_iterator>(std::make_pair(leaves_begin(), leaves_end()));
}

quadtree::node::node(const bbox2f_t& bbox, const std::vector<vec2f_t>& points,
                     const indices_t& subset, const params_t& params,
                     uint32_t depth)
    : bbox_(bbox) {
    if (depth >= params.max_depth ||
        subset.size() <= params.max_points_per_cell) {
        indices_ = subset;
    } else {
        vec2f_t center = bbox.center();
        for (uint32_t i = 0; i < 4; ++i) {
            indices_t sub_indices;
            for (const auto& idx : subset) {
                if (inside_(i, points[idx], center)) {
                    sub_indices.push_back(idx);
                }
            }
            if (!sub_indices.size()) {
                children_.push_back(nullptr);
                continue;
            }
            // add center and corner to new bbox
            bbox2f_t sub_bbox;
            sub_bbox.extend(center);
            vec2f_t corner((i % 2) ? (bbox.max()[0]) : (bbox.min()[0]),
                           (i / 2) ? (bbox.max()[1]) : (bbox.min()[1]));
            sub_bbox.extend(corner);
            children_.push_back(std::make_shared<node>(
                sub_bbox, points, sub_indices, params, depth + 1));
        }
    }
}

quadtree::node::~node() {}

quadtree::node::indices_t&
quadtree::node::indices() {
    return indices_;
}

const quadtree::node::indices_t&
quadtree::node::indices() const {
    return indices_;
}

std::vector<quadtree::node::ptr_t>&
quadtree::node::children() {
    return children_;
}

const std::vector<quadtree::node::ptr_t>&
quadtree::node::children() const {
    return children_;
}

bool
quadtree::node::inside_(uint32_t quadrant, const vec2f_t& point,
                        const vec2f_t& center) {
    const bool b0 = !std::signbit(point[0] - center[0]);  // true if right
    const bool b1 = !std::signbit(point[1] - center[1]);  // true if top
    return b0 == (quadrant % 2) && b1 == (quadrant / 2);
}

quadtree::node_iterator::node_iterator() : node_() {}

quadtree::node_iterator::node_iterator(node::wptr_t node) : node_(node) {
    if (auto n = node.lock()) {
        for (auto& c : n->children()) {
            if (c) {
                queue_.push_back(node::wptr_t(c));
            }
        }
    }
}

quadtree::node_iterator::~node_iterator() {}

quadtree::node_iterator& quadtree::node_iterator::operator++() {
    if (queue_.empty()) {
        node_ = quadtree::node::wptr_t();
        return *this;
    }
    node_ = queue_.front();
    queue_.pop_front();
    update_queue_();
    return *this;
}

bool quadtree::node_iterator::operator==(const node_iterator& other) {
    return node_.lock() == other.node_.lock();
}

bool quadtree::node_iterator::operator!=(const node_iterator& other) {
    return !this->operator==(other);
}

quadtree::node_iterator::operator bool() const { return !(node_.lock()); }

quadtree::node& quadtree::node_iterator::operator*() {
    if (auto n = node_.lock()) {
        return *n;
    }
    throw std::runtime_error("Dereferencing invalid node_iterator");
}

const quadtree::node& quadtree::node_iterator::operator*() const {
    if (auto n = node_.lock()) {
        return *n;
    }
    throw std::runtime_error("Dereferencing invalid node_iterator");
}

quadtree::node* quadtree::node_iterator::operator->() {
    if (auto n = node_.lock()) {
        return n.get();
    }
    throw std::runtime_error("Dereferencing invalid node_iterator");
}

const quadtree::node* quadtree::node_iterator::operator->() const {
    if (auto n = node_.lock()) {
        return n.get();
    }
    throw std::runtime_error("Dereferencing invalid node_iterator");
}

void
quadtree::node_iterator::update_queue_() {
    if (auto n = node_.lock()) {
        for (auto& c : n->children()) {
            if (c) {
                queue_.push_back(node::wptr_t(c));
            }
        }
    }
}

quadtree::leaf_iterator::leaf_iterator() : quadtree::node_iterator() {}

quadtree::leaf_iterator::leaf_iterator(node::wptr_t node)
    : quadtree::node_iterator(node) {
    this->operator++();
}

quadtree::leaf_iterator::~leaf_iterator() {}

quadtree::leaf_iterator& quadtree::leaf_iterator::operator++() {
    while (auto n = node_.lock()) {
        if (n->children().empty()) break;
        node_iterator::operator++();
    }
    return *this;
}

}  // pcl_compress

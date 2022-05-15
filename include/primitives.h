#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <variant>
#include <vector>

class Point
{
public:
    Point(double x, double y);

    double x() const;
    double y() const;
    double distance(const Point & other) const;

    enum class Quadrant
    {
        First,
        Second,
        Third,
        Fourth
    };

    bool in_quad(const Point & other, Quadrant quad) const;

    bool operator<(const Point & other) const;
    bool operator>(const Point & other) const;
    bool operator<=(const Point & other) const;
    bool operator>=(const Point & other) const;
    bool operator==(const Point & other) const;
    bool operator!=(const Point & other) const;

    friend std::ostream & operator<<(std::ostream & ostream, const Point & point);

private:
    static double sqr(double x);

    constexpr static const double EPS = std::numeric_limits<double>::epsilon();

    double m_x;
    double m_y;
};

class Rect
{
public:
    Rect(const Point & left_bottom, const Point & right_top);

    double xmin() const;
    double ymin() const;
    double xmax() const;
    double ymax() const;
    double distance(const Point & point) const;

    bool contains(const Point & point) const;
    bool intersects(const Rect & other) const;

private:
    Point m_left_bottom;
    Point m_right_top;
};

namespace rbtree {

class PointSet
{
public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;

        reference operator*() const;
        pointer operator->() const;
        iterator & operator++();
        iterator operator++(int);

        friend bool operator==(const iterator & lhs, const iterator & rhs);
        friend bool operator!=(const iterator & lhs, const iterator & rhs);

        friend class PointSet;

    private:
        using data_type = std::variant<std::vector<pointer>, std::set<value_type>::iterator>;

        iterator(std::vector<pointer> && points);
        iterator(std::set<value_type>::iterator && it);

        data_type m_data;
    };

    PointSet() = default;
    PointSet(const std::string & filename);

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    // second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream & ostream, const PointSet & point_set);

private:
    std::set<Point> m_set;
};

inline bool operator==(const rbtree::PointSet::iterator & lhs, const rbtree::PointSet::iterator & rhs)
{
    return lhs.m_data == rhs.m_data;
}

inline bool operator!=(const rbtree::PointSet::iterator & lhs, const rbtree::PointSet::iterator & rhs)
{
    return !(lhs == rhs);
}

inline std::ostream & operator<<(std::ostream & ostream, const rbtree::PointSet & point_set)
{
    ostream << "PointSet(";
    auto begin = point_set.begin();
    for (auto it = begin, end = point_set.end(); it != end; ++it) {
        if (it != begin) {
            ostream << ", ";
        }
        ostream << *it;
    }
    ostream << ")";
    return ostream;
}

} // namespace rbtree

namespace kdtree {

class PointSet
{
    struct Node
    {
        Node(const Point & point);
        Node(const Node & node);

        ~Node() = default;

        void update_rect_by(const std::shared_ptr<Node> & node);
        void update_data();

        Point point;

        Rect rect;

        std::size_t size = 1;

        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;

        std::weak_ptr<Node> parent;
    };

    using NodePtr = Node *;

public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;

        reference operator*() const;
        pointer operator->() const;
        iterator & operator++();
        iterator operator++(int);

        friend bool operator==(const iterator & lhs, const iterator & rhs);
        friend bool operator!=(const iterator & lhs, const iterator & rhs);

        friend class PointSet;

    private:
        using node_t = NodePtr;
        using list_t = std::vector<node_t>;
        using data_type = std::variant<list_t, node_t>;

        iterator(list_t && points);
        iterator(node_t current);

        data_type m_data;
    };

    PointSet() = default;
    PointSet(const std::string & filename);
    PointSet(const PointSet & other);
    PointSet(PointSet && other);

    ~PointSet() = default;

    bool empty() const;
    std::size_t size() const;
    void put(const Point & point);
    bool contains(const Point & point) const;

    std::pair<iterator, iterator> range(const Rect & rect) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream & ostream, const PointSet & point_set);

private:
    static bool less(const Point & lhs, const Point & rhs, bool check_x);
    static bool less(const std::shared_ptr<Node> & lhs, const std::shared_ptr<Node> & rhs, bool check_x);
    static bool balanced(const std::shared_ptr<Node> & root);

    static std::shared_ptr<Node> build_tree(const std::vector<std::shared_ptr<Node>>::iterator & begin,
                                            const std::vector<std::shared_ptr<Node>>::iterator & end,
                                            bool check_x = true);
    static std::shared_ptr<Node> rebuild_tree(std::shared_ptr<Node> & root, bool check_x);

    static std::size_t get_size(const std::shared_ptr<Node> & node);

    // Returns pointer to the nearest NodePtr to the root that is not balanced
    static std::pair<std::shared_ptr<Node> *, bool> insert(std::shared_ptr<Node> & root,
                                                           const Point & point,
                                                           bool check_x = true);

    static NodePtr begin(std::shared_ptr<Node> root);
    static NodePtr next(const NodePtr & node);

    static void set_parents(const std::shared_ptr<Node> & node);

    static void to_vector(const std::shared_ptr<Node> & root, std::vector<NodePtr> & result);
    static void move_to_vector(std::shared_ptr<Node> & root, std::vector<std::shared_ptr<Node>> & result);

    static void range(const std::shared_ptr<Node> & root, const Rect & rect, std::vector<NodePtr> & result);
    static void nearest(const std::shared_ptr<Node> & root,
                        const Point & point,
                        double & current_min,
                        NodePtr & current_ans);

    constexpr static const double alpha = 0.65;

    std::shared_ptr<Node> m_root;
};

inline bool operator==(const kdtree::PointSet::iterator & lhs, const kdtree::PointSet::iterator & rhs)
{
    return lhs.m_data == rhs.m_data;
}

inline bool operator!=(const kdtree::PointSet::iterator & lhs, const kdtree::PointSet::iterator & rhs)
{
    return !(lhs == rhs);
}

inline std::ostream & operator<<(std::ostream & ostream, const kdtree::PointSet & point_set)
{
    ostream << "PointSet(";
    auto begin = point_set.begin();
    for (auto it = begin, end = point_set.end(); it != end; ++it) {
        if (it != begin) {
            ostream << ", ";
        }
        ostream << *it;
    }
    ostream << ")";
    return ostream;
}

} // namespace kdtree

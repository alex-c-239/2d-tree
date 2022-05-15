#include "primitives.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>

Point::Point(double x, double y)
    : m_x(x)
    , m_y(y)
{
}

double Point::x() const
{
    return m_x;
}

double Point::y() const
{
    return m_y;
}

double Point::sqr(double x)
{
    return x * x;
}

double Point::distance(const Point & other) const
{
    return std::sqrt(sqr(m_x - other.m_x) + sqr(m_y - other.m_y));
}

bool Point::in_quad(const Point & other, const Quadrant quad) const
{
    switch (quad) {
    case Quadrant::First:
        return other.m_x > m_x - EPS && other.m_y > m_y - EPS;
    case Quadrant::Second:
        return other.m_x < m_x + EPS && other.m_y > m_y - EPS;
    case Quadrant::Third:
        return other.m_x < m_x + EPS && other.m_y < m_y + EPS;
    case Quadrant::Fourth:
        return other.m_x > m_x - EPS && other.m_y < m_y + EPS;
    default:
        return false;
    }
}

bool Point::operator==(const Point & other) const
{
    return std::abs(m_x - other.m_x) < EPS && std::abs(m_y - other.m_y) < EPS;
}

bool Point::operator!=(const Point & other) const
{
    return !(*this == other);
}

bool Point::operator<(const Point & other) const
{
    return (m_x < other.m_x) || (std::abs(m_x - other.m_x) < EPS && m_y < other.m_y);
}

bool Point::operator>(const Point & other) const
{
    return (m_x > other.m_x) || (std::abs(m_x - other.m_x) < EPS && m_y > other.m_y);
}

bool Point::operator<=(const Point & other) const
{
    return *this < other || *this == other;
}

bool Point::operator>=(const Point & other) const
{
    return *this > other || *this == other;
}

std::ostream & operator<<(std::ostream & ostream, const Point & point)
{
    return ostream << "Point(" << point.m_x << "; " << point.m_y << ")";
}

Rect::Rect(const Point & left_bottom, const Point & right_top)
    : m_left_bottom(left_bottom)
    , m_right_top(right_top)
{
}

double Rect::xmin() const
{
    return m_left_bottom.x();
}

double Rect::ymin() const
{
    return m_left_bottom.y();
}

double Rect::xmax() const
{
    return m_right_top.x();
}

double Rect::ymax() const
{
    return m_right_top.y();
}

bool Rect::contains(const Point & point) const
{
    return m_left_bottom.in_quad(point, Point::Quadrant::First) &&
            m_right_top.in_quad(point, Point::Quadrant::Third);
}

double Rect::distance(const Point & point) const
{
    if (contains(point)) {
        return 0;
    }
    if (m_right_top.in_quad(point, Point::Quadrant::First)) {
        return m_right_top.distance(point);
    }
    if (m_left_bottom.in_quad(point, Point::Quadrant::Third)) {
        return m_left_bottom.distance(point);
    }
    if (m_right_top.in_quad(point, Point::Quadrant::Fourth)) {
        return (point.y() > ymin()) ? (point.x() - xmax()) : point.distance({xmax(), ymin()});
    }
    if (m_left_bottom.in_quad(point, Point::Quadrant::Second)) {
        return (point.y() > ymin()) ? (xmin() - point.x()) : point.distance({xmin(), ymax()});
    }
    return (point.y() > ymax()) ? (point.y() - ymax()) : (ymin() - point.y());
}

bool Rect::intersects(const Rect & other) const
{
    return !(other.ymin() > ymax() || other.ymax() < ymin() || other.xmin() > xmax() || other.xmax() < xmin());
}

rbtree::PointSet::iterator::iterator(std::vector<pointer> && points)
    : m_data(std::forward<std::vector<pointer>>(points))
{
}

rbtree::PointSet::iterator::iterator(std::set<value_type>::iterator && it)
    : m_data(std::forward<std::set<value_type>::iterator>(it))
{
}

rbtree::PointSet::iterator::reference rbtree::PointSet::iterator::operator*() const
{
    return (m_data.index()) ? *std::get<1>(m_data) : *std::get<0>(m_data).back();
}

rbtree::PointSet::iterator::pointer rbtree::PointSet::iterator::operator->() const
{
    return (m_data.index()) ? &*std::get<1>(m_data) : std::get<0>(m_data).back();
}

rbtree::PointSet::iterator & rbtree::PointSet::iterator::operator++()
{
    if (m_data.index() == 0) {
        std::get<0>(m_data).pop_back();
    }
    else {
        ++std::get<1>(m_data);
    }
    return *this;
}

rbtree::PointSet::iterator rbtree::PointSet::iterator::operator++(int)
{
    auto cpy = *this;
    operator++();
    return cpy;
}

rbtree::PointSet::PointSet(const std::string & filename)
{
    std::ifstream file(filename);
    for (double x, y; file >> x;) {
        file >> y;
        m_set.emplace(x, y);
    }
}

bool rbtree::PointSet::empty() const
{
    return m_set.empty();
}

std::size_t rbtree::PointSet::size() const
{
    return m_set.size();
}

void rbtree::PointSet::put(const Point & p)
{
    m_set.insert(p);
}

bool rbtree::PointSet::contains(const Point & p) const
{
    return m_set.find(p) != m_set.end();
}

rbtree::PointSet::iterator rbtree::PointSet::begin() const
{
    return iterator(m_set.begin());
}

rbtree::PointSet::iterator rbtree::PointSet::end() const
{
    return iterator(m_set.end());
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::range(const Rect & rect) const
{
    std::vector<const Point *> result;
    for (const Point & point : m_set) {
        if (rect.contains(point)) {
            result.push_back(&point);
        }
    }
    return {iterator(std::move(result)), iterator()};
}

std::optional<Point> rbtree::PointSet::nearest(const Point & point) const
{
    auto result = std::min_element(m_set.begin(),
                                   m_set.end(),
                                   [&point](const Point & lhs, const Point & rhs) {
                                       return point.distance(lhs) < point.distance(rhs);
                                   });
    return result != m_set.end() ? std::optional<Point>(*result) : std::optional<Point>();
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::nearest(const Point & point, std::size_t k) const
{
    if (k == 0) {
        return {iterator(), iterator()};
    }
    if (m_set.size() <= k) {
        return {begin(), end()};
    }
    std::vector<const Point *> heap(k);
    auto it = m_set.begin();
    for (std::size_t i = 0; i < k; ++i) {
        heap[i] = &(*(it++));
    }
    auto less_dist = [&point](const auto & lhs, const auto & rhs) {
        return point.distance(*lhs) < point.distance(*rhs);
    };
    std::make_heap(heap.begin(), heap.end(), less_dist);
    for (; it != m_set.end(); ++it) {
        if (less_dist(it, heap.front())) {
            std::pop_heap(heap.begin(), heap.end(), less_dist);
            heap.back() = &(*it);
            std::push_heap(heap.begin(), heap.end(), less_dist);
        }
    }
    return {iterator(std::move(heap)), iterator()};
}

kdtree::PointSet::Node::Node(const Point & point)
    : point(point)
    , rect(point, point)
{
}

kdtree::PointSet::Node::Node(const kdtree::PointSet::Node & node)
    : point(node.point)
    , rect(node.rect)
    , left((node.left) ? std::make_shared<Node>(*node.left) : nullptr)
    , right((node.right) ? std::make_shared<Node>(*node.right) : nullptr)
{
}

void kdtree::PointSet::Node::update_rect_by(const std::shared_ptr<Node> & node)
{
    if (!node) {
        return;
    }
    rect = Rect({std::min(rect.xmin(), node->rect.xmin()),
                 std::min(rect.ymin(), node->rect.ymin())},
                {std::max(rect.xmax(), node->rect.xmax()),
                 std::max(rect.ymax(), node->rect.ymax())});
}

std::size_t kdtree::PointSet::get_size(const std::shared_ptr<Node> & node)
{
    return node ? node->size : 0;
}

void kdtree::PointSet::Node::update_data()
{
    size = 1 + get_size(left) + get_size(right);
    rect = Rect(point, point);
    if (!left && !right) {
        return;
    }
    update_rect_by(left);
    update_rect_by(right);
}

kdtree::PointSet::iterator::iterator(list_t && points)
    : m_data(std::forward<list_t>(points))
{
}

kdtree::PointSet::iterator::iterator(node_t current)
    : m_data(current)
{
}

kdtree::PointSet::iterator::reference kdtree::PointSet::iterator::operator*() const
{
    return (m_data.index()) ? std::get<node_t>(m_data)->point : std::get<list_t>(m_data).back()->point;
}

kdtree::PointSet::iterator::pointer kdtree::PointSet::iterator::operator->() const
{
    return (m_data.index()) ? &std::get<node_t>(m_data)->point : &std::get<list_t>(m_data).back()->point;
}

kdtree::PointSet::iterator & kdtree::PointSet::iterator::operator++()
{
    if (m_data.index() == 0) {
        std::get<list_t>(m_data).pop_back();
        return *this;
    }
    auto & node = std::get<node_t>(m_data);
    node = next(node);
    return *this;
}

kdtree::PointSet::iterator kdtree::PointSet::iterator::operator++(int)
{
    iterator cpy = *this;
    operator++();
    return cpy;
}

void kdtree::PointSet::set_parents(const std::shared_ptr<Node> & node)
{
    if (node->left) {
        node->left->parent = node;
        set_parents(node->left);
    }
    if (node->right) {
        node->right->parent = node;
        set_parents(node->right);
    }
}

kdtree::PointSet::PointSet(const kdtree::PointSet & other)
    : m_root((other.m_root) ? std::make_shared<Node>(*other.m_root) : nullptr)
{
    set_parents(m_root);
}

kdtree::PointSet::PointSet(kdtree::PointSet && other)
    : m_root(std::move(other.m_root))
{
}

bool kdtree::PointSet::less(const Point & lhs, const Point & rhs, const bool check_x) // NOLINT
{
    return check_x ? (lhs.x() < rhs.x()) : (lhs.y() < rhs.y());
}

bool kdtree::PointSet::less(const std::shared_ptr<Node> & lhs, const std::shared_ptr<Node> & rhs, const bool check_x) // NOLINT
{
    return less(lhs->point, rhs->point, check_x);
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::build_tree(const std::vector<std::shared_ptr<Node>>::iterator & begin,
                                                                     const std::vector<std::shared_ptr<Node>>::iterator & end,
                                                                     bool check_x)
{
    if (begin == end) {
        return nullptr;
    }
    std::sort(begin,
              end,
              [check_x](const std::shared_ptr<Node> & lhs, const std::shared_ptr<Node> & rhs) {
                  return less(lhs, rhs, check_x);
              });
    auto median = begin + (end - begin) / 2;
    while (median != begin && !less(*(median - 1), *median, check_x)) {
        --median;
    }
    std::shared_ptr<Node> result = std::move(*median);
    result->left = build_tree(begin, median, !check_x);
    result->right = build_tree(median + 1, end, !check_x);
    result->update_data();
    return result;
}

kdtree::PointSet::PointSet(const std::string & filename)
{
    std::ifstream file(filename);
    std::vector<std::shared_ptr<Node>> nodes;
    std::set<Point> points;
    for (double x, y; file >> x;) {
        file >> y;
        points.emplace(x, y);
    }
    nodes.reserve(points.size());
    for (auto & p : points) {
        nodes.push_back(std::make_shared<Node>(p));
    }
    m_root = build_tree(nodes.begin(), nodes.end());
    set_parents(m_root);
}

bool kdtree::PointSet::empty() const
{
    return m_root == nullptr;
}

std::size_t kdtree::PointSet::size() const
{
    return get_size(m_root);
}

bool kdtree::PointSet::balanced(const std::shared_ptr<Node> & root)
{
    return get_size(root->left) <= alpha * get_size(root) &&
            get_size(root->right) <= alpha * get_size(root);
}

void kdtree::PointSet::to_vector(const std::shared_ptr<Node> & root, std::vector<NodePtr> & result)
{
    if (!root) {
        return;
    }
    to_vector(root->left, result);
    result.push_back(root.get());
    to_vector(root->right, result);
}

void kdtree::PointSet::move_to_vector(std::shared_ptr<Node> & root, std::vector<std::shared_ptr<Node>> & result)
{
    if (!root) {
        return;
    }
    move_to_vector(root->left, result);
    result.push_back(std::move(root));
    move_to_vector(result.back()->right, result);
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::rebuild_tree(std::shared_ptr<Node> & root, bool check_x)
{
    std::vector<std::shared_ptr<Node>> nodes;
    move_to_vector(root, nodes);
    return build_tree(nodes.begin(), nodes.end(), check_x);
}

std::pair<std::shared_ptr<kdtree::PointSet::Node> *, bool> kdtree::PointSet::insert(std::shared_ptr<Node> & root,
                                                                                    const Point & point,
                                                                                    bool check_x)
{
    if (!root) {
        root = std::make_unique<Node>(point);
        return {nullptr, false};
    }
    if (root->point == point) {
        return {nullptr, false};
    }
    auto & to = less(point, root->point, check_x) ? root->left : root->right;
    auto [broken, prev_check] = insert(to, point, !check_x);
    root->update_data();
    to->parent = root;
    return balanced(root) ? std::make_pair(broken, prev_check) : std::make_pair(&root, check_x);
}

void kdtree::PointSet::put(const Point & point)
{
    auto [broken, check_x] = insert(m_root, point);
    if (broken != nullptr) {
        auto parent = (*broken)->parent;
        *broken = rebuild_tree(*broken, check_x);
        (*broken)->parent = parent;
        set_parents(*broken);
    }
}

bool kdtree::PointSet::contains(const Point & point) const
{
    std::shared_ptr<Node> current = m_root;
    bool check_x = true;
    while (current) {
        if (current->point == point) {
            return true;
        }
        current = less(point, current->point, check_x) ? current->left : current->right;
        check_x = !check_x;
    }
    return false;
}

void kdtree::PointSet::range(const std::shared_ptr<Node> & root,
                             const Rect & rect,
                             std::vector<NodePtr> & result)
{
    if (!root || !root->rect.intersects(rect)) {
        return;
    }
    if (rect.contains(root->point)) {
        result.push_back(root.get());
    }
    if (root->left && root->left->rect.intersects(rect)) {
        range(root->left, rect, result);
    }
    if (root->right && root->right->rect.intersects(rect)) {
        range(root->right, rect, result);
    }
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::range(const Rect & rect) const
{
    std::vector<NodePtr> result;
    range(m_root, rect, result);
    return {iterator(std::move(result)), iterator()};
}

kdtree::PointSet::NodePtr kdtree::PointSet::begin(std::shared_ptr<Node> root)
{
    if (!root) {
        return {};
    }
    while (root->left) {
        root = root->left;
    }
    return root.get();
}

kdtree::PointSet::iterator kdtree::PointSet::begin() const
{
    return iterator(begin(m_root));
}

kdtree::PointSet::iterator kdtree::PointSet::end() const
{
    return iterator(nullptr);
}

kdtree::PointSet::NodePtr kdtree::PointSet::next(const NodePtr & node) // NOLINT
{
    if (node->right) {
        auto ans = node->right;
        while (ans->left) {
            ans = ans->left;
        }
        return ans.get();
    }
    auto ans = node;
    while (!ans->parent.expired()) {
        auto parent = ans->parent.lock();
        if (parent->left && parent->left.get() == ans) {
            ans = parent.get();
            return ans;
        }
        ans = parent.get();
    }
    return {};
}

void kdtree::PointSet::nearest(const std::shared_ptr<Node> & root,
                               const Point & point,
                               double & current_min,
                               NodePtr & current_ans)
{
    if (!root || root->rect.distance(point) >= current_min) {
        return;
    }
    if (point.distance(root->point) < current_min) {
        current_min = point.distance(root->point);
        current_ans = root.get();
    }
    nearest(root->left, point, current_min, current_ans);
    nearest(root->right, point, current_min, current_ans);
}

std::optional<Point> kdtree::PointSet::nearest(const Point & point) const
{
    double current_min = std::numeric_limits<double>::max();
    NodePtr result = nullptr;
    nearest(m_root, point, current_min, result);
    return (result != nullptr) ? std::optional<Point>(result->point) : std::optional<Point>();
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::nearest(const Point & point, std::size_t k) const
{
    if (k == 0) {
        return {iterator(), iterator()};
    }
    if (k >= size()) {
        return {begin(), end()};
    }
    std::vector<NodePtr> heap(k);
    NodePtr current = begin(m_root);
    for (std::size_t i = 0; i < k; ++i) {
        heap[i] = current;
        current = next(current);
    }
    auto less_dist = [&point](const NodePtr & lhs, const NodePtr & rhs) {
        return point.distance(lhs->point) < point.distance(rhs->point);
    };
    std::make_heap(heap.begin(), heap.end(), less_dist);
    for (; current != nullptr; current = next(current)) {
        if (less_dist(current, heap.front())) {
            std::pop_heap(heap.begin(), heap.end(), less_dist);
            heap.back() = current;
            std::push_heap(heap.begin(), heap.end(), less_dist);
        }
    }
    return {iterator(std::move(heap)), iterator()};
}


#include "PathGraph.h"

#include "../world/Map.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

using namespace OpenRCT2;

class PathGraph
{
};

static uint32_t _lastGenerate;
static PathGraph _pathGraph;

class PathNode
{
public:
    CoordsXYZ coords{};
    std::array<PathNode*, 4> edges;

    size_t GetNumEdges()
    {
        size_t c = 0;
        for (auto edge : edges)
        {
            if (edge != nullptr)
                c++;
        }
        return c;
    }
};

class JunctionNode
{
public:
    CoordsXYZ coords{};
    std::array<JunctionNode*, 4> edges;
};

class TileElementWithCoords
{
public:
    CoordsXYZ coords;
    PathElement* element;

    TileElementWithCoords(CoordsXYZ c, PathElement* el)
        : coords(c)
        , element(el)
    {
    }
};

/*
class QuadTree
{
private:
    static constexpr size_t MAX_CHILDREN = 4;

    struct Node
    {
        std::array<std::unique_ptr<Node>, 4> children;
        std::vector<PathNode*> nodes;
        CoordsRange<CoordsXY> range;
        bool isLeaf = true;

        bool Contains(CoordsXY coords)
        {
            if (coords.x >= range.Point1.x &&
                coords.y >= range.Point1.y &&
                coords.x < range.Point2.x &&
                coords.y < range.Point2.y)
            {
                return true;
            }
            return false;
        }

        void Split()
        {
        }

        bool Add(PathNode* node)
        {
            if (!Contains(node->coords))
                return false;

            if (isLeaf)
            {
                nodes.push_back(node);
                if (nodes.size() > MAX_CHILDREN)
                {
                    Split();
                }
            }
            else
            {
                for (auto& child : children)
                {
                    if (child && child->Add(node))
                    {
                        return true;
                    }
                }
            }
        }
    };
    Node root;

public:
    void Add(PathNode* node)
    {
        root.Add(node);
    }
};
*/

template<typename T> class NodeMap
{
private:
    std::vector<T*> _nodes;

public:
    void Add(T* node)
    {
        _nodes.push_back(node);
    }

    T* Find(CoordsXYZ coords)
    {
        for (auto node : _nodes)
        {
            if (node->coords == coords)
            {
                return node;
            }
        }
        return nullptr;
    }
};

template<typename T> class NodePool
{
public:
    std::vector<T*> nodes;
    NodeMap<T> nodeMap;

    T* Allocate(CoordsXYZ coords)
    {
        auto node = new T();
        nodes.push_back(node);
        node->coords = coords;
        nodeMap.Add(node);
        return node;
    }
};

class GraphBuilder
{
private:
    NodePool<PathNode> _nodes;
    NodePool<JunctionNode> _junctionNodes;

public:
    GraphBuilder()
    {
    }

private:
    static bool IsValidPathZAndDirection(const PathElement* el, int32_t z, Direction d)
    {
        if (el->IsSloped())
        {
            auto slopeDirection = el->GetSlopeDirection();
            if (slopeDirection == d)
            {
                if (z != el->GetBaseZ())
                    return false;
            }
            else
            {
                slopeDirection = direction_reverse(slopeDirection);
                if (slopeDirection != d)
                    return false;
                if (z != el->GetBaseZ() + COORDS_Z_PER_TINY_Z)
                    return false;
            }
        }
        else
        {
            if (z != el->GetBaseZ())
                return false;
        }
        return true;
    }

    static std::optional<TileElementWithCoords> GetConnectedElement(CoordsXYZ loc, const PathElement* srcElement, Direction d)
    {
        loc += CoordsDirectionDelta[d];
        if (srcElement->IsSloped() && srcElement->GetSlopeDirection() == d)
        {
            loc.z += COORDS_Z_PER_TINY_Z;
        }

        auto el = map_get_first_element_at(loc);
        if (el != nullptr)
        {
            do
            {
                if (!el->IsGhost())
                {
                    auto pathEl = el->AsPath();
                    if (pathEl != nullptr && IsValidPathZAndDirection(pathEl, loc.z, d))
                    {
                        return TileElementWithCoords(loc, pathEl);
                    }
                }
            } while (!(el++)->IsLastForTile());
        }
        return {};
    }

    PathNode* Expand(CoordsXYZ coords, PathElement* el)
    {
        auto node = _nodes.nodeMap.Find(coords);
        if (node != nullptr)
            return node;

        node = _nodes.Allocate(coords);
        for (auto d : ALL_DIRECTIONS)
        {
            auto connectedElement = GetConnectedElement(node->coords, el, d);
            if (connectedElement)
            {
                node->edges[d] = Expand(connectedElement->coords, connectedElement->element);
            }
        }

        return node;
    }

    void Expand(CoordsXY coords)
    {
        auto el = map_get_first_element_at(coords);
        if (el != nullptr)
        {
            do
            {
                auto pathElement = el->AsPath();
                if (pathElement != nullptr)
                {
                    Expand(CoordsXYZ(coords, pathElement->GetBaseZ()), pathElement);
                }
            } while (!(el++)->IsLastForTile());
        }
    }

    JunctionNode* ExpandJunction(PathNode* src, PathNode* target)
    {
        auto result = _junctionNodes.nodeMap.Find(target->coords);
        if (result != nullptr)
        {
            return result;
        }

        if (target->GetNumEdges() != 2)
        {
            result = _junctionNodes.Allocate(target->coords);

            // Expand to edges
            for (auto d : ALL_DIRECTIONS)
            {
                auto edge = target->edges[d];
                if (edge != nullptr && edge != src)
                {
                    result->edges[d] = ExpandJunction(target, edge);
                }
            }

            return result;
        }
        else
        {
            // Tail recurse to next connected node
            PathNode* newTarget = nullptr;
            for (auto edge : target->edges)
            {
                if (edge != nullptr && edge != src)
                {
                    newTarget = edge;
                    break;
                }
            }
            assert(newTarget != nullptr);
            return ExpandJunction(target, newTarget);
        }
    }

    void ExpandJunctions()
    {
        for (auto node : _nodes.nodes)
        {
            if (node->GetNumEdges() != 2)
            {
                ExpandJunction(nullptr, node);
            }
        }
    }

public:
    PathGraph Generate()
    {
        for (int32_t y = 0; y < gMapSize; y++)
        {
            for (int32_t x = 0; x < gMapSize; x++)
            {
                Expand(TileCoordsXY(x, y).ToCoordsXY());
            }
        }
        ExpandJunctions();
        return {};
    }
};

void GuestPathfinding::Update()
{
    if (_lastGenerate == 0 || _lastGenerate > 40 * 5)
    {
        GraphBuilder builder;
        _pathGraph = builder.Generate();
    }
    else
    {
        _lastGenerate++;
    }
}

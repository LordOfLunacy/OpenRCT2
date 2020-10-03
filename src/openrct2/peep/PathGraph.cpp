
#include "PathGraph.h"

#include "../ride/Ride.h"
#include "../world/Map.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace OpenRCT2;

class PathGraph
{
};

static uint32_t _lastGenerate;
static PathGraph _pathGraph;

class ParkNavNode;

template<typename T> class PathNodeBase
{
public:
    uint32_t id{};
    CoordsXYZ coords{};
};

struct RideStationId
{
    ride_id_t Ride = RIDE_ID_NULL;
    StationIndex Station = STATION_INDEX_NULL;

    RideStationId() = default;
    RideStationId(ride_id_t ride, StationIndex station)
        : Ride(ride)
        , Station(station)
    {
    }

    friend bool operator==(const RideStationId& lhs, const RideStationId& rhs)
    {
        return lhs.Ride == rhs.Ride && lhs.Station == rhs.Station;
    }

    friend bool operator!=(const RideStationId& lhs, const RideStationId& rhs)
    {
        return !(lhs == rhs);
    }
};

class PathNode : public PathNodeBase<PathNode>
{
public:
    std::array<PathNode*, 4> edges;
    std::array<RideStationId, 4> entrance;
    std::array<RideStationId, 4> exit;
    ParkNavNode* blue;
    bool navDone{};

    uint32_t GetEdgeTarget(size_t index)
    {
        size_t c = 0;
        for (auto edge : edges)
        {
            if (edge != nullptr)
            {
                if (c == index)
                {
                    return edge->id;
                }
                c++;
            }
        }
        return std::numeric_limits<uint32_t>::max();
    }

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

class JunctionNode : public PathNodeBase<JunctionNode>
{
public:
    std::array<JunctionNode*, 4> edges;

    uint32_t GetEdgeTarget(size_t index)
    {
        size_t c = 0;
        for (auto edge : edges)
        {
            if (edge != nullptr)
            {
                if (c == index)
                {
                    return edge->id;
                }
                c++;
            }
        }
        return std::numeric_limits<uint32_t>::max();
    }

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

class ParkNavNode : public PathNodeBase<JunctionNode>
{
public:
    struct Edge
    {
        ride_id_t ViaRide = RIDE_ID_NULL;
        ParkNavNode* Target{};
    };

    PathNode* red{};
    std::vector<Edge> edges;

    bool HasEdgeTo(ParkNavNode* target) const
    {
        for (const auto& edge : edges)
        {
            if (edge.Target == target)
            {
                return true;
            }
        }
        return false;
    }

    uint32_t GetEdgeTarget(size_t index) const
    {
        return edges[index].Target->id;
    }

    size_t GetNumEdges() const
    {
        return edges.size();
    }
};

class ConnectedElement
{
public:
    CoordsXYZ coords;
    TileElement* element;

    ConnectedElement(CoordsXYZ c, TileElement* el)
        : coords(c)
        , element(el)
    {
    }
};

template<typename T> class QuadTree
{
private:
    static constexpr size_t MAX_CHILDREN = 4;

    struct Node
    {
        std::array<std::unique_ptr<Node>, 4> children;
        std::vector<T*> nodes;
        CoordsRange<CoordsXY> range;
        bool isLeaf = true;

        Node(CoordsRange<CoordsXY> r)
            : range(r)
        {
        }

        bool Contains(CoordsXY coords)
        {
            if (coords.x >= range.Point1.x && coords.y >= range.Point1.y && coords.x < range.Point2.x
                && coords.y < range.Point2.y)
            {
                return true;
            }
            return false;
        }

        void Split()
        {
            assert(nodes.size() != 0);

            uint32_t midX{}, midY{};
            for (auto node : nodes)
            {
                midX += node->coords.x;
                midY += node->coords.y;
            }
            midX /= static_cast<uint32_t>(nodes.size());
            midY /= static_cast<uint32_t>(nodes.size());

            children[0] = std::make_unique<Node>(CoordsRange<CoordsXY>(range.Point1.x, range.Point1.y, midX, midY));
            children[1] = std::make_unique<Node>(CoordsRange<CoordsXY>(midX, range.Point1.y, range.Point2.x, midY));
            children[2] = std::make_unique<Node>(CoordsRange<CoordsXY>(range.Point1.x, midY, midX, range.Point2.y));
            children[3] = std::make_unique<Node>(CoordsRange<CoordsXY>(midX, midY, range.Point2.x, range.Point2.y));
            isLeaf = false;

            for (auto node : nodes)
            {
                Add(node);
            }
            nodes = {};
        }

        bool ShouldSplit() const
        {
            if (nodes.size() > MAX_CHILDREN)
            {
                auto size = static_cast<int64_t>(range.Point2.x) - range.Point1.x;
                if (size > 128)
                {
                    return true;
                }
            }
            return false;
        }

        bool Add(T* node)
        {
            if (!Contains(node->coords))
                return false;

            if (isLeaf)
            {
                nodes.push_back(node);
                if (ShouldSplit())
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

            return true;
        }

        T* Find(CoordsXYZ coords)
        {
            if (!Contains(coords))
                return nullptr;

            if (isLeaf)
            {
                for (auto node : nodes)
                {
                    if (node->coords == coords)
                    {
                        return node;
                    }
                }
            }
            else
            {
                for (auto& child : children)
                {
                    if (child)
                    {
                        auto node = child->Find(coords);
                        if (node != nullptr)
                        {
                            return node;
                        }
                    }
                }
            }

            return nullptr;
        }
    };
    Node root = Node(CoordsRange<CoordsXY>(
        std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max(),
        std::numeric_limits<int32_t>::max()));

public:
    void Add(T* node)
    {
        root.Add(node);
    }

    T* Find(CoordsXYZ coords)
    {
        return root.Find(coords);
    }
};

template<typename T> class NodePool
{
public:
    std::vector<T*> nodes;
    QuadTree<T> nodeMap;

    T* Allocate(CoordsXYZ coords)
    {
        auto node = new T();
        nodes.push_back(node);
        node->id = static_cast<uint32_t>(nodes.size());
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
    NodePool<ParkNavNode> _parkNavNodes;

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

    static std::optional<ConnectedElement> GetConnectedElement(CoordsXYZ loc, const PathElement* srcElement, Direction d)
    {
        // Ignore direction if path has no edge to it
        auto edges = srcElement->GetEdges();
        if (!(edges & (1 << d)))
        {
            return {};
        }

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
                if (el->IsGhost())
                    continue;

                switch (el->GetType())
                {
                    case TILE_ELEMENT_TYPE_PATH:
                    {
                        auto pathEl = el->AsPath();
                        if (pathEl != nullptr && IsValidPathZAndDirection(pathEl, loc.z, d))
                        {
                            return ConnectedElement(loc, el);
                        }
                        break;
                    }
                    case TILE_ELEMENT_TYPE_ENTRANCE:
                    {
                        auto entranceEl = el->AsEntrance();
                        if (entranceEl->GetDirection() == d || entranceEl->GetDirection() == direction_reverse(d))
                        {
                            if (entranceEl->GetEntranceType() == ENTRANCE_TYPE_RIDE_ENTRANCE
                                || entranceEl->GetEntranceType() == ENTRANCE_TYPE_RIDE_EXIT)
                            {
                                return ConnectedElement(loc, el);
                            }
                        }
                        break;
                    }
                }
            } while (!(el++)->IsLastForTile());
        }
        return {};
    }

    PathNode* Expand(CoordsXYZ coords, PathElement* el)
    {
        if (el->IsQueue())
            return nullptr;

        auto node = _nodes.nodeMap.Find(coords);
        if (node != nullptr)
            return node;

        node = _nodes.Allocate(coords);
        for (auto d : ALL_DIRECTIONS)
        {
            auto connectedElement = GetConnectedElement(node->coords, el, d);
            if (connectedElement)
            {
                switch (connectedElement->element->GetType())
                {
                    case TILE_ELEMENT_TYPE_PATH:
                    {
                        auto pathEl = connectedElement->element->AsPath();
                        if (pathEl->IsQueue())
                        {
                            // A ride entrance
                            node->entrance[d].Ride = pathEl->GetRideIndex();
                            node->entrance[d].Station = pathEl->GetStationIndex();
                        }
                        else
                        {
                            node->edges[d] = Expand(connectedElement->coords, pathEl);
                        }
                        break;
                    }
                    case TILE_ELEMENT_TYPE_ENTRANCE:
                    {
                        auto entranceEl = connectedElement->element->AsEntrance();
                        if (entranceEl->GetEntranceType() == ENTRANCE_TYPE_RIDE_ENTRANCE)
                        {
                            // A ride entrance
                            node->entrance[d].Ride = entranceEl->GetRideIndex();
                            node->entrance[d].Station = entranceEl->GetStationIndex();
                        }
                        else if (entranceEl->GetEntranceType() == ENTRANCE_TYPE_RIDE_EXIT)
                        {
                            // A ride exit
                            node->exit[d].Ride = entranceEl->GetRideIndex();
                            node->exit[d].Station = entranceEl->GetStationIndex();
                        }
                        break;
                    }
                }
            }
        }

#if DEBUG
        for (auto d1 : ALL_DIRECTIONS)
        {
            for (auto d2 : ALL_DIRECTIONS)
            {
                if (d1 != d2)
                {
                    assert(node->entrance[d1] == RideStationId() || node->entrance[d1] != node->entrance[d2]);
                    assert(node->exit[d1] == RideStationId() || node->exit[d1] != node->exit[d2]);
                }
            }
        }
#endif

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
                if (edge != nullptr)
                {
                    result->edges[d] = ExpandJunction(target, edge);
                }
            }

            assert(result->GetNumEdges() == target->GetNumEdges());
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

    std::optional<RideStationId> GetRideNextStationExit(RideStationId rs)
    {
        const auto& rideManager = GetRideManager();
        const auto ride = rideManager[rs.Ride];
        if (ride != nullptr)
        {
            // TODO move this to ride helper
            for (StationIndex i = 1; i <= MAX_STATIONS; i++)
            {
                auto stationIndex = (rs.Station + i) % MAX_STATIONS;
                const auto& nextStation = ride->stations[stationIndex];
                if (!nextStation.Exit.isNull())
                {
                    return RideStationId(ride->id, stationIndex);
                }
            }
        }
        return {};
    }

    PathNode* FindRideExitNode(RideStationId find)
    {
        for (auto node : _nodes.nodes)
        {
            for (const auto& rs : node->exit)
            {
                if (rs.Ride == find.Ride && rs.Station == find.Station)
                {
                    return node;
                }
            }
        }
        return nullptr;
    }

    void ExpandNav2()
    {
    }

    bool DoesNodeHaveRideAssoc(PathNode* node)
    {
        for (auto rideEntrance : node->entrance)
        {
            if (rideEntrance.Ride != RIDE_ID_NULL)
            {
                return true;
            }
        }
        for (auto rideExit : node->exit)
        {
            if (rideExit.Ride != RIDE_ID_NULL)
            {
                return true;
            }
        }
        return false;
    }

    bool ConnectNavNodes(ParkNavNode* a, ParkNavNode* b)
    {
        if (a == b)
        {
            return false;
        }
        if (a->HasEdgeTo(b))
        {
            return false;
        }

        ParkNavNode::Edge e;
        e.Target = b;
        a->edges.push_back(e);

        e.Target = a;
        b->edges.push_back(e);
        return true;
    }

    void ExpandNav()
    {
        // Create nav node for each ride entrance and exit
        for (auto node : _nodes.nodes)
        {
            if (DoesNodeHaveRideAssoc(node))
            {
                auto navNode = _parkNavNodes.Allocate(node->coords);
                navNode->red = node;
                node->blue = navNode;
            }
        }

        // Now connect them
        for (auto navNode : _parkNavNodes.nodes)
        {
            for (auto rideEntrance : navNode->red->entrance)
            {
                if (rideEntrance.Ride != RIDE_ID_NULL)
                {
                    auto rideExit = GetRideNextStationExit(rideEntrance);
                    if (rideExit)
                    {
                        auto exitNode = FindRideExitNode(*rideExit);
                        if (exitNode != nullptr)
                        {
                            auto target = _parkNavNodes.nodeMap.Find(exitNode->coords);
                            if (target != nullptr)
                            {
                                ParkNavNode::Edge e;
                                e.Target = target;
                                e.ViaRide = rideExit->Ride;
                                navNode->edges.push_back(e);
                            }
                        }
                    }
                }
            }
        }

        // Ensure each node has 2 shortest connections
        for (auto navNode : _parkNavNodes.nodes)
        {
            // Reset visited
            for (auto node : _nodes.nodes)
            {
                node->navDone = false;
            }
            int32_t newConnections = 0;

            std::queue<PathNode*> flood;
            flood.push(navNode->red);
            navNode->red->navDone = true;
            while (!flood.empty())
            {
                auto node = flood.front();
                flood.pop();

                if (node->blue != nullptr)
                {
                    if (ConnectNavNodes(navNode, node->blue))
                    {
                        newConnections++;
                        if (newConnections == 2)
                        {
                            break;
                        }
                    }
                }

                for (auto d : ALL_DIRECTIONS)
                {
                    auto next = node->edges[d];
                    if (next != nullptr && !next->navDone)
                    {
                        next->navDone = true;
                        flood.push(next);
                    }
                }
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
        ExpandNav();
        DumpGraph("graph.graphml");
        return {};
    }

    void DumpGraph(const std::string& path)
    {
        // Dump graph compatible with https://www.yworks.com/yed-live/
        std::stringstream sb;
        sb << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        sb << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\"\n";
        sb << "         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n";
        sb << "         xmlns:y=\"http://www.yworks.com/xml/yfiles-common/3.0\"\n";
        sb << "         xmlns:yjs=\"http://www.yworks.com/xml/yfiles-for-html/2.0/xaml\"\n";
        sb << "         xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n";
        sb << "    <key id=\"d5\" for=\"node\" attr.name=\"NodeGeometry\" "
              "y:attr.uri=\"http://www.yworks.com/xml/yfiles-common/2.0/NodeGeometry\"/>\n";
        sb << "    <key id=\"d7\" for=\"node\" attr.name=\"NodeStyle\" "
              "y:attr.uri=\"http://www.yworks.com/xml/yfiles-common/2.0/NodeStyle\"/>\n";
        sb << "    <graph id=\"G\" edgedefault=\"directed\">\n";
        // for (auto n : _nodes.nodes)
        // {
        //     DumpNode(sb, n, "n", "#FFFF0000");
        // }
        // for (auto n : _junctionNodes.nodes)
        // {
        //     DumpNode(sb, n, "j", "#FF0000FF");
        // }
        for (auto n : _parkNavNodes.nodes)
        {
            DumpNode(sb, n, "n", "#FF0000FF");
        }
        sb << "    </graph>\n";
        sb << "</graphml>\n";
        auto str = sb.str();

        auto f = std::fopen(path.c_str(), "wb");
        if (f != nullptr)
        {
            std::fwrite(str.c_str(), 1, str.size(), f);
            std::fclose(f);
        }
    }

    template<typename T> void DumpNode(std::stringstream& sb, T* n, const std::string& type, const std::string& colour)
    {
        sb << "        <node id=\"" << type << n->id << "\">\n";
        sb << "            <data key=\"d5\">\n";
        sb << "                <y:RectD X=\"" << ((32 * 256) - n->coords.x) << "\" Y=\"" << n->coords.y
           << "\" Width=\"16\" Height=\"16\" />\n";
        sb << "            </data>\n";
        sb << "            <data key=\"d7\">\n";
        sb << "                <yjs:ShapeNodeStyle stroke=\"" << colour << "\" fill=\"" << colour << "\" />\n";
        sb << "            </data>\n";
        sb << "        </node>\n";
        auto numEdges = n->GetNumEdges();
        for (size_t i = 0; i < numEdges; i++)
        {
            auto id = n->GetEdgeTarget(i);
            sb << "        <edge source=\"" << type << n->id << "\" target=\"" << type << id << "\"/>\n";
        }
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

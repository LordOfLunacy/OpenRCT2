
#include "PathGraph.h"

#include "../ride/Ride.h"
#include "../world/Map.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <vector>

using namespace OpenRCT2;

class PathGraph
{
};

static uint32_t _lastGenerate;
static PathGraph _pathGraph;

class JunctionNode;
class ParkNavNode;

template<typename T> class PathNodeBase
{
public:
    uint32_t id{};
    CoordsXYZ coords{};
};

struct RideStationId
{
    RideId Ride = RideId::GetNull();
    StationIndex Station = StationIndex::GetNull();

    RideStationId() = default;
    RideStationId(RideId ride, StationIndex station)
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
    JunctionNode* yellow{};
    ParkNavNode* blue{};
    uint8_t areaSize{};
    bool isParkEntrance{};
    bool navDone{};

    PathNode* GetEdgeTarget(size_t index) const
    {
        size_t c = 0;
        for (auto edge : edges)
        {
            if (edge != nullptr)
            {
                if (c == index)
                {
                    return edge;
                }
                c++;
            }
        }
        return nullptr;
    }

    size_t GetNumEdges() const
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
    PathNode* red{};
    std::array<JunctionNode*, 4> edges;

    JunctionNode* GetEdgeTarget(size_t index)
    {
        size_t c = 0;
        for (auto edge : edges)
        {
            if (edge != nullptr)
            {
                if (c == index)
                {
                    return edge;
                }
                c++;
            }
        }
        return nullptr;
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
        RideId ViaRide = RideId::GetNull();
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

    ParkNavNode* GetEdgeTarget(size_t index) const
    {
        return edges[index].Target;
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
private:
    QuadTree<T> nodeMap;

public:
    std::vector<T*> nodes;

    T* Allocate(CoordsXYZ coords)
    {
        auto node = new T();
        nodes.push_back(node);
        node->id = static_cast<uint32_t>(nodes.size());
        node->coords = coords;
        nodeMap.Add(node);
        return node;
    }

    T* Find(CoordsXYZ coords)
    {
        return nodeMap.Find(coords);
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
                    case TileElementType::Path:
                    {
                        auto pathEl = el->AsPath();
                        if (pathEl != nullptr && IsValidPathZAndDirection(pathEl, loc.z, d))
                        {
                            loc.z = el->GetBaseZ();
                            return ConnectedElement(loc, el);
                        }
                        break;
                    }
                    case TileElementType::Entrance:
                    {
                        auto entranceEl = el->AsEntrance();
                        auto entranceType = entranceEl->GetEntranceType();
                        if (entranceType == ENTRANCE_TYPE_RIDE_ENTRANCE || entranceType == ENTRANCE_TYPE_RIDE_EXIT)
                        {
                            if (entranceEl->GetDirection() == d || entranceEl->GetDirection() == direction_reverse(d))
                            {
                                return ConnectedElement(loc, el);
                            }
                        }
                        else if (entranceType == ENTRANCE_TYPE_PARK_ENTRANCE)
                        {
                            if (entranceEl->GetSequenceIndex() == 0)
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
                else
                {
                    auto entranceElement = el->AsEntrance();
                    if (entranceElement != nullptr && entranceElement->GetEntranceType() == ENTRANCE_TYPE_PARK_ENTRANCE
                        && entranceElement->GetSequenceIndex() == 0)
                    {
                        Expand(CoordsXYZ(coords, entranceElement->GetBaseZ()), entranceElement);
                    }
                }
            } while (!(el++)->IsLastForTile());
        }
    }

    PathNode* Expand(CoordsXYZ coords, EntranceElement* el)
    {
        assert(el->GetEntranceType() == ENTRANCE_TYPE_PARK_ENTRANCE);
        assert(el->GetSequenceIndex() == 0);
        auto node = _nodes.Find(coords);
        if (node != nullptr)
            return node;

        node = _nodes.Allocate(coords);
        node->isParkEntrance = true;
        return node;
    }

    PathNode* Expand(CoordsXYZ coords, PathElement* el)
    {
        if (el->IsQueue())
            return nullptr;

        auto node = _nodes.Find(coords);
        if (node != nullptr)
            return node;

        node = _nodes.Allocate(coords);
        for (auto d : ALL_DIRECTIONS)
        {
            // Edge already set, must have been a footpath
            if (node->edges[d] != nullptr)
            {
                continue;
            }

            auto connectedElement = GetConnectedElement(node->coords, el, d);
            if (connectedElement)
            {
                switch (connectedElement->element->GetType())
                {
                    case TileElementType::Path:
                    {
                        auto pathEl = connectedElement->element->AsPath();
                        if (pathEl->IsQueue())
                        {
                            node->entrance[d].Ride = pathEl->GetRideIndex();
                            node->entrance[d].Station = pathEl->GetStationIndex();
                        }
                        else
                        {
                            auto neighbour = _nodes.Find(connectedElement->coords);
                            if (neighbour != nullptr)
                            {
                                node->edges[d] = neighbour;
                                neighbour->edges[direction_reverse(d)] = node;
                            }
                        }
                        break;
                    }
                    case TileElementType::Entrance:
                    {
                        auto entranceEl = connectedElement->element->AsEntrance();
                        auto entranceType = entranceEl->GetEntranceType();
                        if (entranceType == ENTRANCE_TYPE_PARK_ENTRANCE)
                        {
                            auto neighbour = _nodes.Find(connectedElement->coords);
                            if (neighbour != nullptr)
                            {
                                node->edges[d] = neighbour;
                                neighbour->edges[direction_reverse(d)] = node;
                            }
                        }
                        else if (entranceType == ENTRANCE_TYPE_RIDE_ENTRANCE)
                        {
                            node->entrance[d].Ride = entranceEl->GetRideIndex();
                            node->entrance[d].Station = entranceEl->GetStationIndex();
                        }
                        else if (entranceType == ENTRANCE_TYPE_RIDE_EXIT)
                        {
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

    void ExpandNodes()
    {
        for (int32_t y = 0; y < gMapSize.y; y++)
        {
            for (int32_t x = 0; x < gMapSize.x; x++)
            {
                Expand(TileCoordsXY(x, y).ToCoordsXY());
            }
        }

        // Calculate the width of footpaths
        std::queue<PathNode*> qCurrent, qNext;
        for (auto node : _nodes.nodes)
        {
            node->navDone = false;
            if (node->GetNumEdges() < 4)
            {
                node->areaSize = 0;
                node->navDone = true;
                qNext.push(node);
            }
        }

        int32_t distance = 1;
        while (!qNext.empty())
        {
            qCurrent = std::move(qNext);
            while (!qCurrent.empty())
            {
                auto node = qCurrent.front();
                qCurrent.pop();

                for (auto d : ALL_DIRECTIONS)
                {
                    auto next = node->edges[d];
                    if (next != nullptr && !next->navDone)
                    {
                        next->areaSize = distance;
                        next->navDone = true;
                        qNext.push(next);
                    }
                }
            }
            distance++;
        }
    }

    bool CanBeJunction(const PathNode* node)
    {
        bool edges[4]{};
        PathNode* corners[4][2]{};
        int32_t numEdges = 0;
        for (auto d : ALL_DIRECTIONS)
        {
            auto forward = node->edges[d];
            if (forward != nullptr)
            {
                edges[d] = true;
                corners[d][0] = forward->edges[direction_prev(d)];
                corners[d][1] = forward->edges[direction_next(d)];
                numEdges++;
            }
        }

        int32_t numCorners = 0;
        auto singleEdge = false;
        for (auto d : ALL_DIRECTIONS)
        {
            auto left = corners[d][0] != nullptr && corners[d][0] == corners[direction_prev(d)][1];
            auto right = corners[d][1] != nullptr && corners[d][1] == corners[direction_next(d)][0];
            if (right)
                numCorners++;
            if (edges[d] && !left && !right)
            {
                singleEdge = true;
            }
        }

        // return node->GetNumEdges() != 2 && total <= 4;
        if (node->GetNumEdges() != 2 && singleEdge)
            return true;

        auto total = numEdges + numCorners;
        if (total == 7)
        {
            int32_t numJunctionNeighbours = 0;
            for (auto d : ALL_DIRECTIONS)
            {
                auto neighbour = node->edges[d];
                if (neighbour != nullptr)
                {
                    if (_junctionNodes.Find(neighbour->coords))
                        numJunctionNeighbours++;
                }
            }
            if (numJunctionNeighbours == 0)
                return true;
        }

        return false;
    }

    JunctionNode* FindNextJunction(PathNode* startNode, Direction d)
    {
        // Check if direction goes anywhere
        auto forward = startNode->edges[d];
        if (forward == nullptr)
        {
            return nullptr;
        }

        // Reset visited flags
        for (auto node : _nodes.nodes)
        {
            node->navDone = false;
        }

        // Start search
        std::stack<PathNode*> stack;
        forward->navDone = true;
        stack.push(forward);

        while (!stack.empty())
        {
            auto next = stack.top();
            stack.pop();

            // Is this node a junction?
            auto result = _junctionNodes.Find(next->coords);
            if (result != nullptr)
            {
                return result;
            }

            // Push behind
            auto n = next->edges[direction_reverse(d)];
            if (n != nullptr && !n->navDone)
            {
                n->navDone = true;
                stack.push(n);
            }

            // Push left
            n = next->edges[direction_prev(d)];
            if (n != nullptr && !n->navDone)
            {
                n->navDone = true;
                stack.push(n);
            }

            // Push right
            n = next->edges[direction_next(d)];
            if (n != nullptr && !n->navDone)
            {
                n->navDone = true;
                stack.push(n);
            }

            // Push fowards
            n = next->edges[d];
            if (n != nullptr && !n->navDone)
            {
                n->navDone = true;
                stack.push(n);
            }
        }
        return nullptr;
    }

    void ExpandJunctions()
    {
        std::queue<JunctionNode*> expandQueue;
        for (auto node : _nodes.nodes)
        {
            if (CanBeJunction(node))
            {
                auto jn = _junctionNodes.Allocate(node->coords);
                jn->red = node;
                node->yellow = jn;
                expandQueue.push(jn);
            }
        }

        while (!expandQueue.empty())
        {
            auto jn = expandQueue.front();
            expandQueue.pop();

            // Connect up edges
            for (auto d : ALL_DIRECTIONS)
            {
                auto next = FindNextJunction(jn->red, d);
                if (next != nullptr)
                {
                    jn->edges[d] = next;
                }
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
            for (StationIndex::UnderlyingType i = 0; i < OpenRCT2::Limits::MaxStationsPerRide; i++)
            {
                StationIndex stationIndex = StationIndex::FromUnderlying(
                    (rs.Station.ToUnderlying() + i) % OpenRCT2::Limits::MaxStationsPerRide);
                const auto& nextStation = ride->GetStation(stationIndex);
                if (!nextStation.Exit.IsNull())
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

    bool DoesNodeHaveRideAssoc(PathNode* node)
    {
        for (auto rideEntrance : node->entrance)
        {
            if (!rideEntrance.Ride.IsNull())
            {
                return true;
            }
        }
        for (auto rideExit : node->exit)
        {
            if (!rideExit.Ride.IsNull())
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
            if (node->isParkEntrance || DoesNodeHaveRideAssoc(node) /* || node->areaSize >= 3 */)
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
                if (!rideEntrance.Ride.IsNull())
                {
                    auto rideExit = GetRideNextStationExit(rideEntrance);
                    if (rideExit)
                    {
                        auto exitNode = FindRideExitNode(*rideExit);
                        if (exitNode != nullptr)
                        {
                            auto target = _parkNavNodes.Find(exitNode->coords);
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
        ExpandNodes();
        ExpandJunctions();
        ExpandNav();
        // DumpGraph("graph.graphml");
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
            auto target = n->GetEdgeTarget(i);
            if (target != nullptr)
            {
                auto id = target->id;
                sb << "        <edge source=\"" << type << n->id << "\" target=\"" << type << id << "\"/>\n";
            }
        }
    }

    ScreenCoordsXY ConvertToScreenCoords(const rct_viewport* viewport, const CoordsXYZ& coords)
    {
        auto pos = translate_3d_to_2d_with_z(get_current_rotation(), coords.ToTileCentre());
        pos -= viewport->viewPos;
        pos.x = viewport->zoom.ApplyTo(pos.x);
        pos.y = viewport->zoom.ApplyTo(pos.y);
        return pos;
    }

    void Draw(const rct_viewport* viewport, rct_drawpixelinfo* dpi)
    {
        rct_drawpixelinfo dpi2;
        if (clip_drawpixelinfo(&dpi2, dpi, viewport->pos, viewport->width, viewport->height))
        {
            DrawMiniGraph(viewport, &dpi2, _nodes, PALETTE_INDEX_61, 4);
            DrawMiniGraph(viewport, &dpi2, _junctionNodes, PALETTE_INDEX_51, 6);
            DrawMiniGraph(viewport, &dpi2, _parkNavNodes, PALETTE_INDEX_138, 8);
        }
    }

    template<typename T>
    void DrawMiniGraph(const rct_viewport* viewport, rct_drawpixelinfo* dpi, const T& pool, uint8_t colour, int32_t size)
    {
        // Edges
        for (auto node : pool.nodes)
        {
            auto srcPos = ConvertToScreenCoords(viewport, node->coords);
            auto numEdges = node->GetNumEdges();
            for (size_t i = 0; i < numEdges; i++)
            {
                auto target = node->GetEdgeTarget(i);
                if (target != nullptr)
                {
                    DrawEdge(viewport, dpi, node->coords, target->coords, colour);
                }
            }
        }

        // Nodes
        for (auto node : pool.nodes)
        {
            DrawNode(viewport, dpi, node->coords, colour, size);
        }
    }

    void DrawEdge(const rct_viewport* viewport, rct_drawpixelinfo* dpi, const CoordsXYZ& a, const CoordsXYZ& b, uint8_t colour)
    {
        auto srcPos = ConvertToScreenCoords(viewport, a);
        auto dstPos = ConvertToScreenCoords(viewport, b);
        ScreenLine line{ srcPos, dstPos };
        gfx_draw_line(dpi, line, colour);
    }

    void DrawNode(const rct_viewport* viewport, rct_drawpixelinfo* dpi, const CoordsXYZ& coords, uint8_t colour, int32_t size)
    {
        auto nodeSize = viewport->zoom.ApplyTo(size);
        auto srcPos = ConvertToScreenCoords(viewport, coords);
        ScreenRect rect{ srcPos.x - nodeSize, srcPos.y - nodeSize, srcPos.x + nodeSize, srcPos.y + nodeSize };
        gfx_fill_rect(dpi, rect, PALETTE_INDEX_10);
        rect.Point1.x++;
        rect.Point1.y++;
        rect.Point2.x--;
        rect.Point2.y--;
        gfx_fill_rect(dpi, rect, colour);
    }
};

static GraphBuilder _graphBuilder;

void GuestPathfinding::Update()
{
    if (_lastGenerate == 0 || _lastGenerate > 40 * 5)
    {
        GraphBuilder builder;
        _pathGraph = builder.Generate();
        _graphBuilder = std::move(builder);
        _lastGenerate == 1;
    }
    else
    {
        _lastGenerate++;
    }
}

void GuestPathfinding::Draw(const rct_viewport* viewport, rct_drawpixelinfo* dpi)
{
    _graphBuilder.Draw(viewport, dpi);
}

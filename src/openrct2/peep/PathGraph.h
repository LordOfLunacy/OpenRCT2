#pragma once

struct rct_viewport;
struct rct_drawpixelinfo;

namespace OpenRCT2
{
    namespace GuestPathfinding
    {
        void Update();
        void Draw(const rct_viewport* viewport, rct_drawpixelinfo* dpi);
    }

} // namespace OpenRCT2

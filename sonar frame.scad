include <BOSL2/std.scad>

// Circuit board
board_w   = 46;    // board width (edge to edge)
board_t   = 1;   // board thickness
clearance = 0.2;   // slack so the board slides in

// Holder
holder_h = 23;     // total height
groove   = 1;      // how deep the board edges sit in the grooves
wall     = 1.5;    // plastic around the grooves
back_wall = 2;
base     = 4;      // bottom rail height (board bottom sits at base - groove)

// Frame
frame_size = [121, 108];
frame_h    = 10;
frame_wall = 3;
frame_rounding = 6;  // outer corner radius; inner is reduced by the wall so thickness stays even

// Gussets: from the back of each post down onto the top of the frame wall
gusset_len = frame_wall;  // how far they reach across the frame top
gusset_h   = 10;          // how high they climb the post above the frame (max holder_h - frame_h)

// Zumo mounts: tabs reaching inward from the frame to the bot
bot_w       = 65;    // bot body width, left to right
back_reach  = 10;    // frame inner face to bot's back side
front_glue  = [68, 77];  // bot's back to the start / end of the front glue area
front_glue_z = 5;    // height of the front glue area above the frame bottom
rib_t   = 3;         // side tab thickness
pad_w   = 8;         // glue pad width (along the bot's side)
pad_t   = 2;         // glue pad thickness
rib_fillet = 2.5;    // side tab gusset leg length, at the frame and at the pad
fillet  = 5;         // front tab gusset leg length
plate_t = front_glue_z;  // front tab is solid from the bed up to the glue surface (prints without supports)
plate_w = 30;        // front tab width
plate_pad = [40, front_glue[1] - front_glue[0]];  // front tab glue end: width, depth

frame_isize = frame_size - 2*[frame_wall, frame_wall];
side_reach  = (frame_isize.x - bot_w) / 2;
front_reach = frame_isize.y - back_reach - front_glue[0];  // frame inner face to far edge of glue area
assert(frame_isize.y - back_reach - front_glue[1] >= 0, "back_reach too large: front glue area would be past the frame's front wall");
eps = 0.01;

slot_w = board_w + clearance;      // groove-bottom to groove-bottom
slot_t = board_t + clearance;      // groove width
holder_size = [slot_w + 2*wall, slot_t + wall + back_wall, holder_h];

// U-channel board guide: two grooved posts joined by a grooved bottom rail.
// Attachable, so it works with attach()/align= like a cuboid.
module board_holder(anchor=CENTER, spin=0, orient=UP) {
    attachable(anchor, spin, orient, size=holder_size) {
        diff()
        cuboid(holder_size) {
            // open the middle above the rail, leaving the two posts
            tag("remove") position(TOP) up(0.01)
                cuboid([slot_w - 2*groove, holder_size.y + 1, holder_h - base + 0.01], anchor=TOP);
            // the groove the board slides into
            tag("remove") position(TOP) up(0.01) fwd((back_wall - wall)/2) 
                cuboid([slot_w, slot_t, holder_h - (base - groove) + 0.01], anchor=TOP);
            // triangular braces from the back of each post onto the frame top
            xflip_copy() position(BACK+BOT) right(slot_w/2 + (wall - groove)/2)
                rotate([90, 0, 90]) linear_extrude(wall + groove, center=true)
                    polygon([[-0.01, frame_h - 0.01], [gusset_len, frame_h - 0.01], [-0.01, frame_h + gusset_h]]);
        }
        children();
    }
}

// Triangle in the XY plane with its right angle at the origin, legs along +x and +y.
module fillet2d(a, b) polygon([[0, 0], [a, 0], [0, b]]);

// Upright tab for gluing against the bot's side. Local coords: starts at the frame's
// inner face (y=0), reaches +y by `reach`, sits on z=0, full frame height.
module side_mount(reach) {
    rib_len = reach - pad_t;
    pad_fillet = min(rib_fillet, (pad_w - rib_t) / 2);  // keep the pad-joint gussets within the pad
    linear_extrude(frame_h) {
        translate([-rib_t/2, -eps]) square([rib_t, rib_len + 2*eps]);   // rib
        translate([-pad_w/2, rib_len]) square([pad_w, pad_t]);         // glue pad
        for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0]) {
            translate([rib_t/2 - eps, -eps]) fillet2d(rib_fillet, rib_fillet);   // frame joint
            translate([rib_t/2 - eps, rib_len + eps]) mirror([0, 1]) fillet2d(pad_fillet, pad_fillet);  // pad joint
        }
    }
}

// Flat tab lying on the bed for gluing onto the bot's raised front area. Same local coords.
module front_mount(reach) {
    stem_len = reach - plate_pad.y;
    linear_extrude(plate_t) {
        translate([-plate_w/2, -eps]) square([plate_w, stem_len + 2*eps]);
        translate([-plate_pad.x/2, stem_len]) square(plate_pad);
        for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0])
            translate([plate_w/2 - eps, -eps]) fillet2d(fillet, fillet);
    }
    // upright braces tying the plate to the frame wall
    for (x = [-1, 1] * (plate_w/2 - rib_t/2))
        translate([x + rib_t/2, -eps, 0]) rotate([90, 0, -90])
            linear_extrude(rib_t) polygon([[0, 0], [-2*fillet, 0], [0, frame_h]]);
}

// Frame with a holder on each side
rect_tube(size=frame_size, wall=frame_wall, h=frame_h, anchor=CENTER,
          rounding=frame_rounding, irounding=max(frame_rounding - frame_wall, 0), $fn=48)
    attach([FRONT, BACK, LEFT, RIGHT], BACK, overlap=0.01, align=BOT) board_holder();

// Mounting tabs, all sitting on the frame's bottom plane so the part prints flat
down(frame_h/2) {
    left(frame_isize.x/2)  rotate(-90) side_mount(side_reach);
    right(frame_isize.x/2) rotate(90)  side_mount(side_reach);
    back(frame_isize.y/2)  rotate(180) side_mount(back_reach);
    fwd(frame_isize.y/2)               front_mount(front_reach);
}

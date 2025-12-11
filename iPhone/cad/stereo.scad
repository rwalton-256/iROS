spacing = 140; // distance between cases
case_width = 78;
case_height = 168;

corner_size = 10; // size of corner cutout in mm

module iphone_case_with_cutouts() {
    difference() {
        import("iphone_case.stl");
        translate([-(case_width+3)/2, -case_height/2, -1]) cube([corner_size, corner_size, 20]);
        translate([(case_width+3-corner_size*2)/2, -case_height/2, -1]) cube([corner_size, corner_size, 20]);
        translate([-(case_width+3)/2, case_height/2-corner_size, -1]) cube([corner_size, corner_size, 20]);
        translate([(case_width+3-corner_size*2)/2, case_height/2-corner_size, -1]) cube([corner_size, corner_size, 20]);
        translate([-(case_width+10)/2, 10, -1]) cube([corner_size, 50, 20]);
        translate([(case_width+3-corner_size*2)/2, -40, -1]) cube([corner_size, 2*corner_size, 20]);
        translate([(case_width+3-corner_size*2)/2+1, 10, -1]) cube([corner_size, 70, 20]);

        translate([-5,10, -1]) cube([12, 12, 20]);
        translate([-5,30, -1]) cube([12, 12, 20]);
        translate([-5,-10, -1]) cube([12, 12, 20]);
        translate([-5,-30, -1]) cube([12, 12, 20]);
        translate([-5,-50, -1]) cube([12, 12, 20]);
        translate([-5,-70, -1]) cube([12, 12, 20]);
        translate([-25,10, -1]) cube([12, 12, 20]);
        translate([-25,30, -1]) cube([12, 12, 20]);
        translate([-25,50, -1]) cube([12, 12, 20]);
        translate([-25,-10, -1]) cube([12, 12, 20]);
        translate([-25,-30, -1]) cube([12, 12, 20]);
        translate([-25,-50, -1]) cube([12, 12, 20]);
        translate([-25,-70, -1]) cube([12, 12, 20]);
        translate([15,10, -1]) cube([12, 12, 20]);
        translate([15,30, -1]) cube([12, 12, 20]);
        translate([15,50, -1]) cube([12, 12, 20]);
        translate([15,-10, -1]) cube([12, 12, 20]);
        translate([15,-30, -1]) cube([12, 12, 20]);
        translate([15,-50, -1]) cube([12, 12, 20]);
        translate([15,-70, -1]) cube([12, 12, 20]);
    }

}

translate([0, 0, 0]) iphone_case_with_cutouts();
translate([spacing, 0, 0]) iphone_case_with_cutouts();
translate([case_width/2, 1, 13]) rotate([0, 90, 0]) cube([13,5,spacing-case_width]);
translate([case_width/2, -60, 13]) rotate([0, 90, 0]) cube([13,5,spacing-case_width]);

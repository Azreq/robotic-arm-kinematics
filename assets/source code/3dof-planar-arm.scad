$fn = 50;

L1 = 100;
L2 = 75;
L3 = 60;
width = 20;
height = 10;
joint_radius = 15;

module link_y(length) {
    difference() {
        union() {
            cylinder(r=joint_radius, h=height, center=true);
            translate([-width/2, 0, -height/2])
                cube([width, length, height]);
            translate([0, length, 0])
                cylinder(r=joint_radius, h=height, center=true);
        }
        cylinder(r=joint_radius/2, h=height+2, center=true);
        translate([0, length, 0])
            cylinder(r=joint_radius/2, h=height+2, center=true);
    }
}

color("dodgerblue")
    translate([0, 0, 0])
        link_y(L1);

color("darkslategray")
    translate([0, L1, height + 2])
        link_y(L2);

color("dodgerblue")
    translate([0, L1 + L2, 2 * height + 4])
        link_y(L3);
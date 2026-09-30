$fn = 50;

rail_length = 150;
L1_slide = 100;
L2 = 75;
width = 20;
height = 10;
joint_radius = 15;

color("blue") translate([-width/2, 0, -height]) 
    cube([width, rail_length, height/2]);

color("darkslategray") translate([-width*0.75, L1_slide - width, -height/2]) 
    cube([width*1.5, width*2, height]);

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

color("blue") translate([0, L1_slide, height + 2]) link_y(L2);

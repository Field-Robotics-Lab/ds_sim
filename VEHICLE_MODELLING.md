# The dsros_hydro plugin, hydrodynamics, and your vehicle

Vehicle dynamics modeling is easy, but explaining it is hard.  There are lots of super-mathy 
explanations that get really far into the details of how to compute
forces and accelerations at any one time.  There are some decent references in various places-- 
Thor Fossen's "Handbook of Marine Craft Hydrodynamics and Motion Control" has been very helpful
when writing this plugin.  However, there are a few things Fossen glosses over or that have
explainations that are far more detailed than necessary.  This document attempts to provide a simpler
introduction and pointers on how to get your vehicle model into the general vehicle model framework.

The dsros_hydro plugin is intended to model a single underwater vehicle that remains fully submerged.
There is extremely crude support for surface interactions, mostly intended to keep vehicles in the water.
The goal of the plugin is to provide a general interface to plug in your vehicle model at an adequate
level of sophistication while remaining tractable to use.  Notably, the dsros_hydro plugin does not
assume the vehicle is described in its center of mass frame.

In general, the dsros_hydro plugin models the following forces:

 * Vehicle inertia
 * Vehicle weight, applied at center of mass
 * Buoyant force, applied at center of bouyancy
 * Linear and Quadratic Drag forces, applied at center of drag
 * Added mass damping, applied at center of mass
 
There are a very large number of parameters here.  A full vehicle model can comprise up to 335.  Where
possible, a default parameter of 0 is assumed.  This makes it especially easy to specify decoupled models
in the config file.

Gazebo's internal physics engine is kind of a mess.  Anyone looking at the implementation will notice
that different forces are stored internally in all kinds of different reference frames.  This is because
Gazebo's underlying physics engine sometimes assumings forces happen to an object's center of mass,
sometimes applies forces to the object's origin, and almost never bothers to document which is which.
If you ever have to modify the plugin, the only way to know what's going on is to read the gazebo 
source with a copy of the ODE manual.  It's a sorry state of affairs.

## Vehicle Weight, Mass, and Inertia

Vehicle inertia is handled through gazebo's internal physics engine (usually ODE).  Mass and inertia are 
loaded through the standard `inertial` URDF or SDF tags.  Here's a sample from an Xacro file:
```xml
<link name="your_vehicles_base_link">
    <!-- other stuff -->
    <inertial>
        <origin xyz="${com_x} ${com_y} ${com_z}" rpy="0 0 0"/>
    	<mass value="453.592"/>
    	<inertia ixx="46.5860609943583" ixy="0" ixz="0"
    		             iyy="114.624724916758" iyz="0"
                                 izz="110.483572365747"/>
    </inertial>
    <!-- also possibly other stuff -->
</link>
```
The origin tag is used to specify the center of mass.  The center of mass is specified, in meters, 
in the reference frame of the vehicle's base TF link.  Mass value is given in kilograms.  Moment of 
inertia is given in SI-standard kg m^2.  Note that the moment of inertia matrix is symmetric, so 
the bottom half of the matrix is automatically filed in to preserve that.

All coriolis and centripetal effects due to the effects of using a reference frame not at the 
vehicle's center of mass are handled internally by Gazebo.

## Buoyant Force

Buoyancy is modeled as a simple force, always against gravity, applied at the specified center of 
buoyancy.  The center of buoyancy is specified as kg displacement, and the center of buoyancy is 
described in meters relative to the base link.  As the force is applied by the hydro plugin,
the buoyancy parameters are included in the plugin tag block.  For example:

```xml
<plugin name="dsros_hydro" filename="libdsros_hydro.so">
    <base_link>base_link</base_link>

    <buoyancy>
        <!-- This is the center-of-bouyancy -->
        <center x="${cob_x}" y="${cob_y}" z="${cob_z}"/>
        <mass>453.592</mass>

        <!-- This is TOTAL system buoyancy -->
        <buoyancy>455.592</buoyancy>

        <!-- Above out_of_water, we assume no bouyancy -->
        <out_of_water_depth>${0*in_to_m}</out_of_water_depth>

        <!-- Below fully-submerged, we assume full buoyancy -->
        <fully_submerged_depth>${8*in_to_m}</fully_submerged_depth>

        <!-- In between, it's linear.  But its also helpful to apply a scale factor. -->
        <scale_factor>1.0</scale_factor>
    </buoyancy>

    <drag>
        <!-- More on this in the next section -->
    </drag>
</plugin>
```

The surface interaction is extremely crude, and is intended mostly for completeness.  When deeper than the fully 
submerged depth, full buoyancy is applied.  When shallower than the out-of-water-depth, no buoyancy is applied.
In between, a portion of the full buoyancy, determined by the scale factor, is applied.  In general, the difference
between out_of_water depth and fully_submerged_depth, and the scale factor, seem to have very little impact on settling
time.

## Hydrodynamic Drag

Arriving at a hydrodynamic model from a vehicle design is, in general, hard.  This plugin ignores 
all of that, and strives to simply implement the model that must otherwise be supplied.

The drag force is broken down into three components: a linear drag term, where force is proportional to 
velocity, a quadratic drag term, where force is proportional to the square of velocity, and an added
mass term, where hydrodynamic forces mimic inertial effects.  Linear and quadratic drag
 are applied at the specified center of drag, while added mass is applied at the center of mass.

All the drag terms are included in a single "drag" tag in the plugin as follows:

```xml
<plugin name="dsros_hydro" filename="libdsros_hydro.so">
    <base_link>base_link</base_link>

    <buoyancy>
        <!-- Covered in previous section -->
    </buoyancy>

    <drag>
        <!-- This is the center of drag.  Probably the center-of-mass, or thereabouts-->
        <center x="${cod_x}" y="${cod_y}" z="${cod_z}"/>
        <linear>
            <!-- More on this in a second -->
        </linear>

        <quadratic>
            <!-- More shortly -->
        </quadratic>
 
        <added_mass>
            <!-- Coefficients will go here -->
        </added_mass>
    </drag>
</plugin>
```

### Linear Drag

As the name implies, linear drag is proportional to speed.  Given the 6-DOF linear+angular velocity
vector v, the total 6-DOF force+torque vector is T = L v, where L is the 6 x 6 linear drag matrix.
Note that this matrix allows velocity in any degree of freedom to couple into force in another degree
of freedom.

The linear drag matrix is specifed with tags of the form `<aa>`, where `a` specifies an index in L 
corresponding to a given axis.  The axes are `x`, `y`, `z`, `r`, `p`, `h` for surge, sway, heave, 
roll, pitch, and heading, respectively.  Any elements not included are assumed to be zero.  Here's
an example that uses a decoupled model, i.e., it only specifies the diagonal elements:

```xml
<linear>
    <xx>coeff_surge</xx>
    <yy>coeff_sway</yy>
    <zz>coeff_heave</zz>
    <rr>coeff_roll</rr>
    <pp>coeff_pitch</pp>
    <hh>coeff_heading</hh>
</linear>
```

Another way to think about these tags is that the first character in the tag specifies the axis the 
force is applied to, while the second character specifies the velocity causing that force.  So,
`<xy>` gives the coefficient of force generated in the x-direction by velocity in the y-direction; 
that is, Fx = Lxy * Vy.  Note that the density of water and all other constants must be included in the 
provided coefficients.

A decoupled model like that shown is typically sufficient.

### Quadratic Drag

Remember how the 6-DOF velocities needed a 36-element vector to map to the 6-DOF force vector?
Quadratic drag takes in two 6-DOF velocities and maps them to a 6-DOF force vector.  So it needs a
matrix-like thing that's 6 x 6 x 6.  There's probably a tensor of some order that very neatly 
describes all that, but our goal here is to avoid getting bogged down in math.  Instead, let's extend
the indexing approach we introduced back with linear drag with one minor addition.

In order to work properly, quadratic drag requires a sign-preserving square operation-- typically
written as the product of the absolute value of velocity times velocity, or with scalars as 
Fd = Cd *|v|*v.  Because of this, the quadratic drag tensor isn't *quite* symmetric, and in 
theory may need to be fully-specified.

Therefore, we extend the previous matrix approach used for linear drag by adding another character
to the tag.  As before, the first character determines the axis of the force applied.  The second
character determines the axis of the absolute value of velocity.  The third character determines
the axis of the signed velocity.  Or, in summary, coefficient Qabc generates a force from 
velocity as Fa = Qabc * |Vb| * Vc.  The order of the sign was chosen to be consistent with
the convention of applying signed velocity last (see Steve Martin's dissertation).

As with linear drag, it is often reasonable to assume a decoupled model with the only non-zero terms
being clustered on the diagonal.  For example:

```xml
<quadratic>
    <xxx>coeff_surge</xxx>
    <yyy>coeff_sway</yyy>
    <zzz>coeff_heave</zzz>
    <rrr>coeff_roll</rrr>
    <ppp>coeff_pitch</ppp>
    <hhh>coeff_heading</hhh>
</quadratic>
```

### Added Mass

Added mass is the result of hydrodynamic forces that act as if additional mass is imparted on the vehicle.
It is related to the need to accelerate water as a submerged body accelerates, but in practice I find it
more helpful to think of it as a term in the equation of motion that gets fit to real data.  Added mass
is parameterized by a 6 x 6 added mass matrix.  Although positive semi-definite, Fossen notes that in 
practice all 36 elements can be distinct.  As with many other such parameters, the simulator implements
all 36 elements and assumes all unspecified elements are zero.  A Coriolis-Centripital matrix is 
computed at each timestep using the derivation in Fossen's _Handbook of Marine Craft Hydrodynamics
and Motion Control_, Page 120, Equations 6.43 - 6.46.  Also much like the other drag matrices, Fossen
notes that fully-submerged marine vehicles with three axes of symmetry can typically ignore the 
off-diagonal elements of the added mass matrix.  

The added-mass matrix is specified much like the linear drag matrix.  The elements are:

```xml
<added_mass>
    <xx>mass_kg</xx>
    <yy>mass_kg</yy>
    <zz>mass_kg</zz>
    <rr>Ixx</rr>
    <pp>Iyy</pp>
    <hh>Izz</hh>
</added_mass>
```

In the absence of data, a good rule of thumb is to use the vehicle's mass inertia matrix.  This effectively makes
the added mass equal to the vehicle's actual mass.

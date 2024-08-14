

#include <GL/glew.h>
#include <random>
#include <functional>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"

#include <cmath>


#include <string>
#include <sstream>
#include "../../lib/mesh/mesh_io.hpp"



using namespace cpe;


static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex(vec3(-L, h,-L));
    m.add_vertex(vec3(-L, h, L));
    m.add_vertex(vec3( L, h, L));
    m.add_vertex(vec3( L, h,-L));

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}


//********************************************//
// Build a skinned cylinder along z axis      //
// radius is the radius of the cylinder       // 
// length is the length of the cylinder       //
// Nu is the number of points for each circle //
// Nv is the number of points in the length   //
//********************************************//
static cpe::mesh_skinned build_cylinder(float const radius,float const length,int const Nu,int const Nv)
{
    cpe::mesh_skinned m;

/* TODO : 
 1) Add Nu*Nv points search as this point belong to the cylinder
 2) Give weights to theses points
 3) Link the points together to form an open cylinder

    Pseudocode :
    for ku in 0:Nu
        u:float <- value in the circle according to ku \in [0,2\pi[ (M_PI is defined)
        for kv in 0:Nv
            v:float <- value in the length according to kv \in [0,length]

            x:float <- x coordinate of the point in the circle in the plane Oxy = radius*cos(u)
            y:float <- y coordinate of the point in the circle in the plane Oxy = radius*sin(u)
            z:float <- z coordinate of the point along the z axis = v

            add vertice (x,y,z) to m
            
            change weights (NOT in the first questions)

    for ku in 0:Nu
        for kv in 0:Nv-1       
            k0:int <- vertex indice of the ku,kv point = Nv*ku+kv
            k1:int <- vertex indice of the next point in the same circle = Nv*((ku+1)%Nu)+kv
            k2:int <- vertex indice of the next point in the next circle = Nv*((ku+1)%Nu)+(kv+1)
            k3:int <- vertex indice of the point in the next circle = Nv*ku+(kv+1)

            add triangle indices for k0, k1, k2
            add triangle indices for k0, k2, k3
*/
    for(int ku=0;ku<Nu;++ku)
    {
        float u=2.0f*M_PI*ku/(Nu-1);
        for(int kv=0;kv<Nv;++kv)
        {
            float v=length*kv/(Nv-1);

            float x=radius*cos(u);
            float y=radius*sin(u);
            float z=v;

            skinning_weight poid0;
            skinning_weight poid1;
            vertex_weight_parameter vertex_list;
            poid0.joint_id = 0;
            poid1.joint_id = 1;
            poid0.weight = 1-kv/(Nv-1.0f);
            poid1.weight = kv/(Nv-1.0f);
            vertex_list[0] = poid0;
            vertex_list[1] = poid1;


            m.add_vertex({x,y,z});
            m.add_vertex_weight(vertex_list);
        }
    }
    for(int ku=0;ku<Nu;++ku)
    {
        for(int kv=0;kv<Nv-1;++kv)
        {
            int k0=Nv*ku+kv;
            int k1=Nv*((ku+1)%Nu)+kv;
            int k2=Nv*((ku+1)%Nu)+(kv+1);
            int k3=Nv*ku+(kv+1);

            m.add_triangle_index({k0,k1,k2});
            m.add_triangle_index({k0,k2,k3});
        }
    }




    return m;
}

void scene::create_skeleton(const float length){
    sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);
    skeleton_joint sj({0,0,0},quaternion(0,0,0,1));
    sk_cylinder_bind_pose.push_back(sj);
    sj.position.z() = length/2;
    sk_cylinder_bind_pose.push_back(sj);
    sj.position.z() = length/2;
    sk_cylinder_bind_pose.push_back(sj);
}

 cpe::skeleton_animation scene::animation_skeleton(skeleton_geometry bind_pose){
    float angle_rotation=M_PI/6;
    for(int k=0;k<4;k++){
        cpe::skeleton_joint j0;
        cpe::skeleton_joint j1;
        cpe::skeleton_joint j2;
        cpe::quaternion q;
        cpe::skeleton_geometry sk_g_tmp;
        j0.position = bind_pose[0].position;
        j1.position = bind_pose[1].position;
        j2.position = bind_pose[2].position;

        j0.orientation = bind_pose[0].orientation;
        j1.orientation.set_axis_angle({0,1,0},k*angle_rotation);
        j2.orientation = bind_pose[2].orientation;

        sk_g_tmp.push_back(j0);
        sk_g_tmp.push_back(j1);
        sk_g_tmp.push_back(j2);
//        std::cout<<"sk_g_tmp : "<<sk_g_tmp<<std::endl;
//        skeleton_geometry const sk_cylinder_global = local_to_global(sk_g_tmp);
//        std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
        //        draw_skeleton();
        sk_cylinder_animation.push_back(sk_g_tmp);
//        draw_skeleton(sk_cylinder_animation);
    }
    return sk_cylinder_animation;
}

void scene::load_scene()
{


    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");           PRINT_OPENGL_ERROR();
    shader_skeleton = read_shader("shaders/shader_skeleton.vert",
                                  "shaders/shader_skeleton.frag");       PRINT_OPENGL_ERROR();


    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(100.0f , -25.0f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);

    float const length = 50.0f;
    float const radius = 4.0f;
    mesh_cylinder = build_cylinder(radius,length,20,30);
    mesh_cylinder.fill_empty_field_by_default();
    mesh_cylinder_opengl.fill_vbo(mesh_cylinder);
    create_skeleton(length);

    mesh_monster.load("data/Monster.obj");
    mesh_monster.fill_empty_field_by_default();
    mesh_monster_opengl.fill_vbo(mesh_monster);
    texture_monster = load_texture_file("data/Monster.png");
    sk_monster_bind_pose.load("data/Monster.skeleton");
    sk_monster_parent_id.load("data/Monster.skeleton");
    sk_monster_animation.load("data/Monster.animations", sk_monster_parent_id.size());
//    std::cout<<sk_monster_animation.size()<<std::endl;

    timer.start();



}



void scene::draw_scene()
{

    setup_shader_skeleton(shader_skeleton);

    skeleton_animation sk_cyl_anim = animation_skeleton(sk_cylinder_bind_pose);

    float frame = 10.0f; // for the monster


//    float frame = 300.0f; // for cylinder

//    float alpha = timer.elapsed()/frame;
    if(timer.elapsed()>=frame){
//        alpha = 0;
        timer.restart();
        current_position = (current_position+1)%sk_monster_animation.size();
    }
//    skeleton_geometry const sk_cylinder_global = local_to_global(sk_cyl_anim[current_position],sk_cylinder_parent_id);
    skeleton_geometry const sk_cylinder_global = local_to_global(sk_cyl_anim(current_position,timer.elapsed()/frame),sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);

//    skeleton_animation sk_monster_anim = animation_skeleton(sk_monster_bind_pose);
    skeleton_geometry const sk_monster_global = local_to_global(sk_monster_animation(current_position,timer.elapsed()/frame),sk_monster_parent_id);
    std::vector<vec3> const sk_monster_bones = extract_bones(sk_monster_global,sk_monster_parent_id);


//    draw_skeleton(sk_cylinder_bones); //draw different skeleton with animations
    //Here we can draw skeletons as 3D segments


    setup_shader_mesh(shader_mesh);
    //for the cylinder
//    mesh_ground_opengl.draw();    //draw the ground
    skeleton_geometry const sk_cylinder_inverse_bind_pose =
    inversed(local_to_global(sk_cylinder_bind_pose,sk_cylinder_parent_id));
    skeleton_geometry const sk_cylinder_binded =
    multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose);
    mesh_cylinder.apply_skinning(sk_cylinder_binded);
    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);
//    mesh_cylinder_opengl.draw();  //draw the cylinder

    //for the mesh
//    mesh_monster_opengl.draw();     //draw the monster
    skeleton_geometry const sk_monster_inverse_bind_pose =
    inversed(local_to_global(sk_monster_bind_pose,sk_monster_parent_id));
    skeleton_geometry const sk_monster_binded =
    multiply(sk_monster_global,sk_monster_inverse_bind_pose);
    mesh_monster.apply_skinning(sk_monster_binded);
    mesh_monster.fill_normal();
    mesh_monster_opengl.update_vbo_vertex(mesh_monster);
    mesh_monster_opengl.update_vbo_normal(mesh_monster);
    mesh_monster_opengl.draw();     //draw the monster



}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
//    glBindTexture(GL_TEXTURE_2D,texture_default); //texture white                                      PRINT_OPENGL_ERROR();
    glBindTexture(GL_TEXTURE_2D,texture_monster); //texture of the monster
    glLineWidth(1.0f);                                                                                 PRINT_OPENGL_ERROR();

}

void scene::setup_shader_skeleton(GLuint shader_id, float r , float g, float b )
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
  glUniform3f(get_uni_loc(shader_id,"color") , r,g,b);                                      PRINT_OPENGL_ERROR();

    //size of the lines
    glLineWidth(3.0f);                                                                                 PRINT_OPENGL_ERROR();
}

void scene::draw_skeleton(std::vector<vec3> const& positions) const
{
    // Create temporary a VBO to store data
    GLuint vbo_skeleton=0;
    glGenBuffers(1,&vbo_skeleton);                                                                     PRINT_OPENGL_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER,vbo_skeleton);                                                        PRINT_OPENGL_ERROR();

    // Update data on the GPU
    glBufferData(GL_ARRAY_BUFFER , sizeof(vec3)*positions.size() , &positions[0] , GL_STATIC_DRAW);    PRINT_OPENGL_ERROR();

    // Draw data
    glEnableClientState(GL_VERTEX_ARRAY);                                                              PRINT_OPENGL_ERROR();
    glVertexPointer(3, GL_FLOAT, 0, 0);                                                                PRINT_OPENGL_ERROR();
    glDrawArrays(GL_LINES,0,positions.size());                                                         PRINT_OPENGL_ERROR();

    // Delete temporary VBO
    glDeleteBuffers(1,&vbo_skeleton);                                                                  PRINT_OPENGL_ERROR();
}

scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}



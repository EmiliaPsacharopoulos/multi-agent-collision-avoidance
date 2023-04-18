from MAPF import cbs_wrapper

cbs_wrapper = cbs_wrapper.CBS_WRAPPER()
cbs_wrapper.set_map("MAPF/instances/demo_easy_2.txt")
path = cbs_wrapper.solve(show_animation=True)
print(path)
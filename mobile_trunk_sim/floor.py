def Floor(parentNode, name,  translation, color=[0.5, 0.5, 0.5, 1.]):

    floor = parentNode.addChild(name)
    floor.addObject('MeshSTLLoader', name='loader', filename='meshes/Assembly.stl', 
                                    rotation=[-270, 0, 90],scale=2000, translation=translation )
    floor.addObject('MeshTopology', src='@loader')
    floor.addObject('OglModel', name="renderer", src='@loader', color=color)
    floor.addObject('MechanicalObject')
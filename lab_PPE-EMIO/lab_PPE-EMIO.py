def createScene(rootnode):

    from emio.utils.header import addHeader, addSolvers
    from emio.parts.controllers.assemblycontroller import AssemblyController
    from emio import Emio

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual", "showInteractionForceFields"]

    # Units are: s, mm, kg
    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["blueleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["counterclockwiseup", "clockwiseup", "counterclockwiseup", "clockwiseup"],
                centerPartName="yellowpart",
                centerPartType="rigid",
                extended=False)
    if not emio.isValid():
        return

    simulation.addChild(emio)
    addSolvers(emio, rayleighMass=0, rayleighStiffness=0)

    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)
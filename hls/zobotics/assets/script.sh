 
#!/bin/bash

# Boucle sur tous les fichiers .stl du dossier courant
for file in ./*.stl; do
    # Récupère le nom du fichier sans extension
    filename=$(basename "$file" .stl)

    # Convertit en .obj dans le même dossier
    meshlabserver -i "$file" -o "./$filename.obj"

    echo "Converti : $file -> $filename.obj"
done

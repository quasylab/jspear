/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.unicam.quasylab.jspear.speclang.types;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class JSpearTypeTest {

    private final JSpearType customType = new JSpearCustomType("testType", new String[] { "element1", "element2" , "element3"} );
    private final JSpearType theSameCustomType = new JSpearCustomType("testType", new String[] { "element1", "element2" , "element3"} );

    private final JSpearType[][] UNMEARGEABLE_TYPES = {
            {  customType, JSpearType.BOOLEAN_TYPE },
            {  customType, JSpearType.INTEGER_TYPE },
            {  customType, JSpearType.REAL_TYPE },
            {  customType, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            {  customType, new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            {  customType, new JSpearRandomType(JSpearType.REAL_TYPE) },
            {  JSpearType.INTEGER_TYPE, JSpearType.BOOLEAN_TYPE },
            {  JSpearType.INTEGER_TYPE, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            {  JSpearType.REAL_TYPE, JSpearType.BOOLEAN_TYPE },
            {  JSpearType.REAL_TYPE, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            {  JSpearType.BOOLEAN_TYPE, JSpearType.INTEGER_TYPE },
            {  JSpearType.BOOLEAN_TYPE, JSpearType.REAL_TYPE },
            {  JSpearType.BOOLEAN_TYPE, new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            {  JSpearType.BOOLEAN_TYPE, new JSpearRandomType(JSpearType.REAL_TYPE) },
    };

    private final JSpearType[][] MEARGEABLE_TYPES = {
            //CUSTOM_TYPE
            {  customType, theSameCustomType, customType },
            {  customType, new JSpearRandomType(theSameCustomType), new JSpearRandomType(customType) },
            //INTEGER_TYPE
            { JSpearType.INTEGER_TYPE, JSpearType.INTEGER_TYPE , JSpearType.INTEGER_TYPE},
            { JSpearType.INTEGER_TYPE, JSpearType.REAL_TYPE , JSpearType.REAL_TYPE},
            { JSpearType.INTEGER_TYPE, new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            { JSpearType.INTEGER_TYPE, new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            //REAL_TYPE
            { JSpearType.REAL_TYPE, JSpearType.INTEGER_TYPE , JSpearType.REAL_TYPE},
            { JSpearType.REAL_TYPE, JSpearType.REAL_TYPE , JSpearType.REAL_TYPE},
            { JSpearType.REAL_TYPE, new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            { JSpearType.REAL_TYPE, new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            //BOOLEAN_TYPE
            { JSpearType.BOOLEAN_TYPE, JSpearType.BOOLEAN_TYPE , JSpearType.BOOLEAN_TYPE},
            { JSpearType.BOOLEAN_TYPE, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) , new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            //ARRAY_TYPE
            //RANDOM[INTEGER_TYPE]
            { new JSpearRandomType(JSpearType.INTEGER_TYPE), JSpearType.INTEGER_TYPE , new JSpearRandomType(JSpearType.INTEGER_TYPE)},
            { new JSpearRandomType(JSpearType.INTEGER_TYPE), JSpearType.REAL_TYPE , new JSpearRandomType(JSpearType.REAL_TYPE)},
            { new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            { new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            //RANDOM[INTEGER_TYPE]
            { new JSpearRandomType(JSpearType.REAL_TYPE), JSpearType.INTEGER_TYPE , new JSpearRandomType(JSpearType.REAL_TYPE)},
            { new JSpearRandomType(JSpearType.REAL_TYPE), JSpearType.REAL_TYPE , new JSpearRandomType(JSpearType.REAL_TYPE)},
            { new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.INTEGER_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            { new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE), new JSpearRandomType(JSpearType.REAL_TYPE) },
            //RANDOM[BOOLEAN_TYPE]
            {  new JSpearRandomType(JSpearType.BOOLEAN_TYPE) , JSpearType.BOOLEAN_TYPE, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            {  new JSpearRandomType(JSpearType.BOOLEAN_TYPE) , new JSpearRandomType(JSpearType.BOOLEAN_TYPE) , new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            //RANDOM[ARRAY_TYPE]
    };

    private final JSpearType[][] COMPATIBLE_TYPES = {
            {JSpearType.BOOLEAN_TYPE, JSpearType.BOOLEAN_TYPE},
            {JSpearType.BOOLEAN_TYPE, new JSpearRandomType(JSpearType.BOOLEAN_TYPE) },
            {JSpearType.INTEGER_TYPE, JSpearType.INTEGER_TYPE},
            {JSpearType.INTEGER_TYPE, new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            {JSpearType.REAL_TYPE, JSpearType.INTEGER_TYPE},
            {JSpearType.REAL_TYPE, JSpearType.REAL_TYPE},
            {JSpearType.REAL_TYPE, new JSpearRandomType(JSpearType.INTEGER_TYPE) },
            {JSpearType.REAL_TYPE, new JSpearRandomType(JSpearType.REAL_TYPE) },
            {customType, theSameCustomType},
            {customType, new JSpearRandomType(theSameCustomType)}
    };

    @Test
    public void testTypesThatShouldBeMerged() {
        for (JSpearType[] mergeable_type : MEARGEABLE_TYPES) {
            assertTrue(mergeable_type[0].canBeMergedWith(mergeable_type[1]), mergeable_type[0] + " should be merged with " + mergeable_type[1]);
        }
    }

    @Test
    public void testTypesThatCannotBeMerged() {
        for (JSpearType[] incompatible_types : UNMEARGEABLE_TYPES) {
            assertFalse(incompatible_types[0].canBeMergedWith(incompatible_types[1]), incompatible_types[0] + " should not be merged with " + incompatible_types[1]);
        }
    }

    @Test
    public void testMergingTypesResults() {
        for (JSpearType[] mergeable_type : MEARGEABLE_TYPES) {
            assertEquals(mergeable_type[2], mergeable_type[0].merge(mergeable_type[1]), mergeable_type[0] + ".merge("+ mergeable_type[1]+")");
        }
    }

    @Test
    public void testErrorMergingTypesResults() {
        for (JSpearType[] incompatibleType : UNMEARGEABLE_TYPES) {
            assertEquals(JSpearType.ERROR_TYPE, incompatibleType[0].merge(incompatibleType[1]), incompatibleType[0] + ".merge("+ incompatibleType[1]+")");
        }
    }

    @Test
    public void testSubtyping() {
        for (JSpearType[] subtypes : COMPATIBLE_TYPES) {
            assertTrue(subtypes[0].isCompatibleWith(subtypes[1]), subtypes[0] + ".isCompatibleWith("+ subtypes[1]+")");
        }
    }

}